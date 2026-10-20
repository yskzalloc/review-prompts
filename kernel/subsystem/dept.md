# DEPT Subsystem Details

DEPT (DEPendency Tracker, `CONFIG_DEPT`) is a runtime deadlock detector that
tracks **wait → event** dependencies instead of lock acquisition order. It
covers primitives lockdep cannot model (folio lock, `PG_writeback`,
completions, waitqueues, swait, wait_bit, DMA fences, RCU grace periods)
and also re-tracks the ordinary locks through the lockdep wrappers.

Load this guide for patches that touch:
- the engine: `kernel/dependency/`, `include/linux/dept.h`,
  `include/linux/dept_sdt.h`, `include/linux/dept_ldt.h`
- annotations in primitives: `include/linux/completion.h`,
  `include/linux/wait.h`, `include/linux/swait.h`,
  `include/linux/wait_bit.h`, `include/linux/dma-fence.h`,
  `drivers/dma-buf/dma-fence.c`, `include/linux/page-flags.h`,
  `include/linux/pagemap.h`, `mm/filemap.c`, `mm/mmu_notifier.c`,
  `include/linux/rcupdate_wait.h`, `kernel/rcu/update.c`,
  the `ldt_*` glue in `include/linux/lockdep.h`, the scheduler hooks in
  `kernel/sched/core.c`, and the `dept_update_cxt()` call sites
- any patch whose changelog quotes a `DEPT: Circular dependency has been
  detected` report, whether it changes code or annotations

Reference documents: `Documentation/dev-tools/dept.rst` (model, report
format) and `Documentation/dev-tools/dept_api.rst` (API list).

## The Model: Wait, Event, Event Context, Class

Misreading the model leads reviewers to accept fixes for cycles that cannot
happen, or to reject real deadlocks because "no lock order is violated".

- A **wait** is a point where a context may block until some event occurs.
- An **event** is what unblocks that wait: an unlock, `folio_unlock()`,
  `folio_end_writeback()`, `complete()`, or any `try_to_wake_up()`.
- An **event context** is the stretch of a context during which it is
  responsible for producing an event. It starts at `[S]` and ends at the
  event `[E]`.
- A **dependency** `A → B` is recorded when a context, inside the event
  context of class A, waits for class B. It means "event A cannot happen
  until event B happens".
- The graph is built over **classes**, never over objects. A report is
  printed only when a newly added edge closes a cycle (`check_dl_bfs()`
  from `add_dep()`). Each edge is added once per boot (`lookup_dep()`), so
  the same report does not repeat. To confirm a fix, reboot and re-run the
  reproducer.

DEPT records dependencies in two ways. Most correctness questions depend on
which one an annotation uses.

| Mechanism | How `[S]` is known | When the edge is added | Used by |
|---|---|---|---|
| Held event context | explicit `dept_ecxt_enter()` / `sdt_ecxt_enter()`, pushed on `dept_task()->ecxt_held[]` | at **wait** time, against every held ecxt (`add_wait()`) | all `ldt_*` lock annotations, `mmu_notifier` invalidate range |
| Wait-generation window | timestamp: `dept_request_event()` stores a global `wgen` in the map (or in a `struct dept_ext_wgen`) | at **event** time: every wait in the event task's own history newer than that `wgen` and in the same context id (`do_event()`) | `sdt_wait()`/`sdt_event()`, staged sleeps + `try_to_wake_up()`, `PG_locked`, `PG_writeback` |

The "window" case explains the `[S] (unknown)(...)` / `(N/A)` lines in
reports. DEPT knows only the timestamp, not the code location where the
obligation started. `(N/A)` is expected there and is not a DEPT bug.

**The model only sees annotated primitives.** A custom wait loop built on
`schedule()`, a polled flag, `msleep()` retry loops, or a raw bitop
protocol is invisible. If DEPT says nothing, first check that every
primitive in the suspected cycle is annotated. Absence of a report is not
proof that there is no cycle.

**Window-based edges are learned only when the event fires.** If the first
time a path runs it actually deadlocks, the event never fires and no edge
is added. Window-based detection needs an earlier run that did *not*
deadlock. For example, the lkdtm `FOLIO_LOCK_AA` scenario is not reported
because the folio is never unlocked. The `FOLIO_MUTEX_LOCK_ABBA` scenario
is reported because the mutex side uses a held ecxt.

## Class Identity and Key Granularity

Key granularity determines how accurate a report is. If two unrelated wait
channels share a key, their edges merge into one graph node and DEPT
reports cycles that cannot happen. If every object gets its own key,
dependencies never accumulate across objects and DEPT goes blind.

**How the class is chosen** (see `check_new_class()` and `sub_id()` in
`kernel/dependency/dept.c`):
- class identity = `key->base + sub_id`, where the key is `m->keys` if
  non-NULL, otherwise `&m->map_key`, the map's own embedded key
- `sub_id = m->sub_u + e * DEPT_MAX_SUBCLASSES_USR`, where `sub_u` is the
  init-time subclass (`< DEPT_MAX_SUBCLASSES_USR`, currently 8) and `e` is
  the event bit index (`< DEPT_MAX_SUBCLASSES_EVT`, currently 3). An
  out-of-range `sub_u` makes `dept_map_init()` warn and mark the map
  `nocheck`. The map is then silently untracked.
- a map whose key is `&__lockdep_no_validate__.dkey` is `nocheck`
  (`valid_key()`). `lockdep_set_novalidate_class()` therefore disables DEPT
  for that lock as well.

**Where a `static struct dept_key` lives defines the granularity.**
`sdt_might_sleep_start_timeout(NULL, t)` and `sdt_map_init(m)` declare a
`static struct dept_key __key` inside the macro body, so there is one key
per **macro expansion site**. A `static` inside a function, including a
`static inline` function, is one object per function (per translation unit
for `static inline`). It is not one per caller, and inlining does not
create copies.

This is why `wait_for_completion*()`, `dma_fence_wait*()` and
`dma_fence_default_wait()` were turned into macros that expand the
annotation at the caller. The out-of-line bodies were renamed to
`__wait_for_completion*()` and `__dma_fence_*()`. It is also why a helper
function that wraps an annotated macro funnels all of its callers into one
class.

```c
// WRONG: every caller of foo_wait() shares the one key expanded inside
// this function, so waits for unrelated channels become one <sched> class
static void foo_wait(struct foo *f)
{
	wait_for_completion(&f->done);
}

// CORRECT: expand the key at each caller and attach it through a map
// (struct foo gains a struct dept_map dmap)
void __foo_wait(struct foo *f, struct dept_key *k)
{
	sdt_map_init_key(&f->dmap, k);
	init_completion_dmap(&f->done, &f->dmap);
	start_foo(f);			/* eventually complete(&f->done) */
	wait_for_completion(&f->done);	/* class taken from f->dmap */
}

#define foo_wait(f)							\
do {									\
	static struct dept_key __key;	/* one per foo_wait() caller */	\
	__foo_wait(f, &__key);						\
} while (0)
```

This is the same pattern as `_wait_rcu_gp()` and `__wait_rcu_gp()` in
`include/linux/rcupdate_wait.h` and `kernel/rcu/update.c`.

The real instance of this bug was `__wait_rcu_gp()`. `synchronize_rcu()` and
`synchronize_rcu_tasks_trace()` both blocked on the single callsite inside
that function, which closed a fake `cpu_hotplug_lock → <sched> →
cpu_hotplug_lock` cycle. The fix ("rcu/update: fix same dept key collision
between various types of RCU") moved `static struct dept_key __key` into the
`_wait_rcu_gp()` macro and passed it down to
`sdt_map_init_key()`/`init_completion_dmap()`. Every `_wait_rcu_gp()`
expansion is now its own class. The rcu_tasks, rcu_tasks_rude and
rcu_tasks_trace flavours still share the one expansion in
`synchronize_rcu_tasks_generic()`.

**Per-instance classes are rarely right.** `dept_map_init(m, NULL, ...)`
and `DEPT_MAP_INITIALIZER(n, NULL)` key the class by `&m->map_key`, which
is the object's own address.
- This is fine for static objects (`pg_locked_map`, `pg_writeback_map`,
  static mutex initializers).
- For on-stack or short-lived objects, it creates a new class on every
  call, so nothing accumulates. When the address is reused, unrelated
  objects alias into a stale class. `dept_map_copy()` refuses `keys ==
  NULL` for exactly this reason: the copy may live on the stack.
- For dynamically allocated objects, the memory must go through
  `dept_free_range()` before it is freed or reused. Otherwise the class
  survives and aliases with the next object at that address.

**Lock classes.** For locks, the key is `lock_class_key::dkey` (see
`lockdep_init_map_type()` → `ldt_init()`), so a lockdep class and a DEPT
class match one to one. Two kinds of subclass behave differently:
- The init-time subclass (`sub_u`) creates distinct classes.
- The acquire-time subclass (`mutex_lock_nested()`, `spin_lock_nested()`)
  becomes the local `sub_l`. It does **not** create a new class. It only
  suppresses the edge between held and waiting instances of the *same*
  class that carry different `sub_l` (`add_wait()`). Do not assume
  lockdep's subclass semantics carry over.

**Event-bit subclasses.** `w_f`/`e_f` select up to
`DEPT_MAX_SUBCLASSES_EVT` sub-ids per map. A wait may set several bits. An
event must set exactly one bit (`DEPT_WARN_ON(1UL << e != e_f)`). The
page-usage scheme relies on this (see the page-flag section).

**REPORT as bugs**:
- A patch that wraps an annotated wait (`wait_event*()`,
  `wait_for_completion*()`, `dma_fence_wait*()`, `sdt_might_sleep_start()`)
  in a shared out-of-line or `static inline` helper used for semantically
  different channels, unless it also passes a per-caller key or map.
- `dept_map_init(m, NULL, ...)` or `DEPT_MAP_INITIALIZER(n, NULL)` on stack
  or heap objects without a matching `dept_free_range()`.

## Staged Waits and try_to_wake_up() Attribution

Most primitives annotate a *region* that might sleep, not the wait itself.
An unbalanced region leaves a stale stage on the task. The next unrelated
`schedule()` then commits it, and whoever wakes the task is credited as the
producer. The result is spurious edges and false positive reports.

How it works (`dept_stage_wait()`, `dept_request_event_wait_commit()`,
`dept_ttwu_stage_wait()`, `dept_clean_stage()`):
1. `sdt_might_sleep_start[_timeout](m, t)` stages a map on the current
   task. With `m == NULL`, the class comes from the callsite key and
   prints as `<sched>`.
2. `__schedule()` commits the stage as a real wait only for a voluntary
   sleep: `sched_mode != SM_PREEMPT && prev_state & TASK_NORMAL`.
   Preemption, spinning, and polling never commit a wait.
3. `try_to_wake_up()` on that task calls `dept_ttwu_stage_wait()`. The
   **waker** fires the event of the staged class and the stage is cleared.
4. `sdt_might_sleep_end()` clears the stage if no wake-up came.

Rules:
- `sdt_might_sleep_start*()` and `sdt_might_sleep_end()` must pair on
  **every** exit path. That includes `goto`, `break`, and early `return`
  inside statement-expression macros. "dept: fix a bug missing
  sdt_might_sleep_end() in swait/wait/wait_bit" moved `sdt_might_sleep_end()`
  below the `__out:` label of `___wait_event()`, `___swait_event()` and
  `___wait_var_event()` because the condition-already-true path jumped
  over it.
- Nesting is not counted. The **outermost** `dept_stage_wait()` wins (inner
  calls return early while `stage_m.keys` is set), and the **first**
  `sdt_might_sleep_end()`, even an inner one, clears the whole stage. An
  explicit outer map around a `wait_event()` (as in
  `mmu_interval_read_begin()`) works only because nothing sleeps between
  the inner end and the outer end.
- The stage is consumed by the first wake-up. Any `try_to_wake_up()`
  counts: the intended `complete()`/`wake_up()`, a signal,
  `wake_up_process()` from an unrelated subsystem, or a spurious wake-up.
  Whatever the waker was blocked on before waking becomes part of the
  edge. This is the most common source of `<sched>`-class false positives.
- For precision, give the channel an explicit map (`sdt_map_init()`,
  `init_completion_dmap()`) and bracket the producer's responsibility with
  `sdt_ecxt_enter()`/`sdt_ecxt_exit()`. When the waker holds an ecxt for
  the staged map, `do_event()` skips the timestamp window. Only waits
  inside the bracketed region count, which is how the `mmu_notifier`
  invalidate-range false positive was fixed.
- `init_completion()` and `init_completion_map()` both resolve to
  `init_completion_dmap(x, NULL)`. The lockdep map passed to
  `init_completion_map()` is ignored by DEPT.

## Timestamp Windows and Context Boundaries

A window that is too long pulls unrelated waits into an event context and
causes false positives. A missing request makes DEPT ignore the event and
miss real dependencies.

- `dept_request_event()` stores a fresh `wgen`. `dept_event()` uses it and
  then resets it to 0. An event with `wgen == 0` is ignored ("event
  triggered before wait").
- `dept_map_reinit()` and `dept_stage_wait()` reset `wgen` to 0. Re-arming
  must happen after them.
- Only waits in the event task's own history count, and only those with
  the same **context id** (`cur_ctxt_id()`). The process context id
  changes on every kernel entry from user mode (`enter_from_user_mode()`
  in `include/linux/irq-entry-common.h`) and on every work item
  (`process_one_work()`). The outermost hardirq and softirq each get their
  own ids. A kthread that loops forever outside a workqueue never starts a
  new context, so its window is bounded only by the history ring.
  `dept_update_cxt()` is the existing way to cut a context; check that a
  new call site really starts an independent unit of work.
- The per-task history is a ring of `DEPT_MAX_WAIT_HIST` (64) entries.
  When it wraps, older waits are lost and `DEPT_INFO_ONCE("Need to expand
  the ring buffer.")` is printed. Consecutive waits of the same class in
  the same context collapse into one entry (`add_hist()`).
- The held-ecxt stack holds at most `DEPT_MAX_ECXT_HELD` (48) entries.
  Overflow warns and drops the ecxt. Enter/exit imbalance is tracked in
  `dept_task()->missing_ecxt` and is not otherwise reported.

## Lock Annotations Through the Lockdep Wrappers

DEPT inherits lockdep's annotations, including their mistakes. An
annotation that tells lockdep "this is an acquire" is, to DEPT, also "this
may wait". Assert-only annotations therefore create real-looking edges.

- The `ldt_*` hooks are called from `spin_acquire*()`, `rwlock_acquire*()`,
  `seqcount_acquire*()`, `mutex_acquire*()`, `rwsem_acquire*()` and their
  release macros in `include/linux/lockdep.h`.
- `lock_map_acquire*()`, `lock_map_release()`, `lock_map_sync()` and direct
  `lock_acquire()`/`lock_release()` calls are **not** wired to DEPT.
  Lockdep-only pseudo-locks are invisible to DEPT. Examples are the
  workqueue/flush maps, the `dma_fence` signalling map, and
  `__mmu_notifier_invalidate_range_start_map`.
- `ldt_lock(m, sl, try, nest, ip)`:
  - normal acquire: `dept_wait()` then `dept_ecxt_enter()`
  - trylock: `dept_ecxt_enter()` only, with no wait
  - `nest_lock`: `dept_ecxt_enter_nokeep()` only
  An annotation meant only to make `lockdep_assert_held()` work must use
  the trylock form. "cpu/hotplug: use a weaker annotation in AP thread"
  changed `rwsem_acquire(..., 0, 0, ...)` to `rwsem_acquire(..., 0, 1,
  ...)` for this reason.
- `rwsem_acquire_read()` calls `ldt_lock()`, the same as a writer. DEPT
  does not model rwsem readers as shared, so a cycle made only of read
  acquisitions of a rwsem or percpu-rwsem (`down_read()`,
  `percpu_down_read()`, `cpus_read_lock()`) is not a real deadlock unless
  a writer is part of the cycle. rwlock readers use `ldt_rlock()` and are
  modelled with read/write event bits.
- `lock_set_class()` maps to `ldt_set_class()` and `lock_downgrade()` maps
  to `ldt_downgrade()`. Both go through `dept_map_ecxt_modify()`, which
  re-keys or changes the event bit of the held ecxt.
- `lockdep_set_lock_cmp_fn()` is honoured only against the **innermost**
  held ecxt (`lockdep_cmp_fn()` in `add_wait()`), through
  `dept_map::lockdep_map`.
- `lockdep_copy_map()` also copies the DEPT map (`dept_map_copy()`).
- `dept_enter()` calls `dept_off()`, which calls `lockdep_off()`. Lockdep
  assertions check `lockdep_recursing_current()` so they do not fire while
  DEPT is running ("locking/lockdep: prevent various lockdep assertions
  when lockdep_off()'ed").

## Page Flag Tracking (PG_locked, PG_writeback)

Folio lock and writeback are the main targets of DEPT and also the
easiest annotations to get subtly wrong. A missed set or clear either
drops the edge or stretches the window across unrelated work.

- There is one global map per bit (`pg_locked_map`, `pg_writeback_map` in
  `mm/filemap.c`). The per-page state is only a `wgen`, stored in
  `struct page::pg_locked_wgen` / `pg_writeback_wgen` (`struct
  dept_ext_wgen`) to keep `struct page` small.
- Annotation contract, documented above `dept_page_set_bit()` in
  `include/linux/page-flags.h`:
  - `dept_page_set_bit()` right after a **successful** first set (only
    when a `test_and_set*` returned 0). This calls `dept_request_event()`.
  - `dept_page_clear_bit()` at the clear, before the wake-up. This calls
    `dept_event()`.
  - `dept_page_wait_on_bit()` at the wait. This calls `dept_wait()`.
  All three must name the **same** `struct page` (the folio head). A
  `wgen` written to one page and read from another is always 0, so the
  event is silently ignored.
- `folio_lock()`, `lock_page()` and `folio_lock_killable()` call
  `dept_page_wait_on_bit()` **before** the trylock. Even uncontended locking
  is treated as a potential wait, so a report does not require contention
  to have happened. `folio_trylock()` records no wait. Converting a
  `folio_lock()` into `folio_trylock()` plus a retry outside the
  conflicting lock really removes the edge. The hugetlb fix
  ("mm,hugetlb: change mechanism to detect a COW on private mapping")
  relies on this.
- With lock hand-off (`WQ_FLAG_CUSTOM` in `wake_page_function()`,
  `folio_trylock_flag()`), the waker sets the bit for the waiter.
  `folio_wait_bit_common()` therefore calls `dept_page_set_bit()` again
  after `WQ_FLAG_DONE`, so the new owner's order is always wait → set →
  clear. Any new hand-off path needs the same re-arm.
- Any new code that sets or clears `PG_locked`/`PG_writeback` with raw
  bitops (`test_and_set_bit_lock()`, `folio_xor_flags_has_waiters()`,
  open-coded `clear_bit_unlock()`) bypasses the `FOLIO_*_FLAG` macro
  annotations and must call `dept_page_*_bit()` itself. Both
  `folio_unlock()` and `folio_end_read()` do this. `folio_wake_bit()`
  calls clear again, which is harmless.
- All folios share one class per bit. DEPT cannot distinguish two folio
  instances. An ABBA between two folio locks shows up as a self-dependency
  on `pg_locked_map` (reported as `*** AA DEADLOCK ***` or not at all). It
  cannot be told apart from a real AA. Treat any report built only from
  folio-lock edges as unproven until the instances are identified.
- Page usage (`dept_set_folio_usage()`, `DEPT_PAGE_REGFILE_CACHE`,
  `DEPT_PAGE_BDEV_CACHE`) splits that class through event bits:
  - waits set `DEFAULT | usage`, events set only `usage`, so regular-file
    and block-device page-cache pages never interact
  - a requested usage stays pending until the next flag operation
    (`dept_update_page_usage()`) and is not applied in the middle of a
    `PG_locked` section
  - in this tree nothing calls `dept_set_folio_usage()` yet, so every page
    is `DEPT_PAGE_DEFAULT`
- The `dept_page_*` helpers act only on `PG_locked` and `PG_writeback`.
  Tracking another bit needs a new map, a new `dept_ext_wgen` in `struct
  page`, initialization in `__init_single_page()`, and a clear annotation
  at its end point (cf. `folio_end_writeback_no_dropbehind()`).

## Timeout Waits

A wait with a finite timeout cannot block forever, so by default DEPT
ignores it. Code changes that only exist to make DEPT quiet can therefore
hide real cycles.

- A wait counts as a timeout wait when `0 < t < MAX_SCHEDULE_TIMEOUT`
  (`dept_wait()`, `dept_stage_wait()`). `-1L`, `0`, and
  `MAX_SCHEDULE_TIMEOUT` count as unbounded.
- `___wait_event()`, `___swait_event()` and `___wait_var_event()` pass
  their `ret` argument as the timeout. A new wrapper that passes a
  positive `ret` that is *not* a timeout changes how the wait is
  classified.
- With `CONFIG_DEPT_AGGRESSIVE_TIMEOUT_WAIT=n` (the default), timeout
  waits are not recorded. With it set, a cycle that includes a timeout
  edge prints `NOT A DEADLOCK BUT A CIRCULAR DEPENDENCY`
  (`is_timeout_wait_circle()`).
- Converting an unbounded wait into a timeout wait silences DEPT without
  fixing anything. It is acceptable only when the timeout is the intended
  semantics and the changelog says so.

## Engine Internals (kernel/dependency/)

The engine runs inside lock, IRQ-flag and scheduler paths. Any recursion,
allocation or ordinary locking in it can deadlock the machine or silently
turn DEPT off.

- Entry points bracket their work with `dept_enter()`/`dept_exit()`, which
  use `arch_local_irq_save()`, increment `dept_task()->recursive` and call
  `lockdep_off()`. Paths that may legitimately re-enter use
  `dept_enter_recursive()`. Engine code must not call instrumented
  primitives such as `spin_lock()`, `mutex_lock()`, waitqueues, or
  `kmalloc()`. The global graph lock is an `arch_spinlock_t` taken by
  trylock (`dept_lock()`), which gives up while a report is printing
  (`dept_outworld_*`). A failed `dept_lock()` silently drops the edge.
- Objects come from static and per-CPU pools (`dept_object.h`,
  `from_pool()`). Refill is deferred to `irq_work` and then a workqueue
  (`dept_wq_work_fn()`). A pool that cannot be extended triggers
  `DEPT_STOP()`.
- `dept_working()` is false in NMI and after `dept_stop` is set.
  `__nmi_enter()` calls `dept_off()`. `debug_locks_off()` calls
  `dept_stop_emerg()`, so **any lockdep splat or oops turns DEPT off for
  the rest of the boot**. DEPT being silent after an earlier lockdep report
  means nothing.
- A map initialized while DEPT is not working (`dept_map_init()`,
  `dept_map_copy()`) is marked `nocheck` permanently.
- Class lifetime:
  - module unload calls `dept_free_range()` on module memory, because
    classes are keyed by addresses and names inside it
  - `lockdep_register_key()`/`lockdep_unregister_key()` call
    `dept_key_init()`/`dept_key_destroy()`
  - both destruction paths end in `synchronize_rcu()` and must be callable
    from a sleepable context
  - `dept_key_init()` on memory that still has live classes, or a
    `dept_free_range()` while `recursive` is set, calls `DEPT_STOP()`
- IRQ dependencies (`add_iecxt()`/`add_iwait()`) need accurate
  `hardirqs_enabled`/`softirqs_enabled` per task, fed from
  `lockdep_hardirqs_on_prepare()`, `lockdep_hardirqs_off()` and
  `lockdep_softirqs_on/off()`. `local_irq_disable()` and
  `local_irq_save()` call `dept_hardirqs_off()` even when IRQs are already
  off, because assembly may have disabled them without tracing. New
  IRQ-state paths must keep these flags right, otherwise IRQ-related
  cycles are missed or invented.
- Functions called from entry or `noinstr` code (`dept_hardirqs_off()`,
  `dept_hardirq_enter()`, `dept_update_cxt()`, `dept_off()`, `dept_on()`)
  must stay `noinstr`.

## Side Effects of CONFIG_DEPT on Unrelated Code

DEPT enlarges core structures, so `CONFIG_DEPT=y` exposes build-time
assumptions that never fail otherwise.

- `struct lockdep_map` embeds a `struct dept_map`, `struct
  lock_class_key` embeds a `struct dept_key`, `struct page` gains `usage`
  and two `dept_ext_wgen`s, `struct completion` gains a `dmap` pointer,
  and `struct rcu_synchronize` gains a map and a key. Every lock-bearing
  structure grows.
- Known breakages of this kind:
  - `kfree_rcu()`'s `offsetof(rcu_head) < 4096` `BUILD_BUG_ON` (fixed by
    moving `rcu` to the front of `struct rpc_xprt`)
  - `PERCPU_DYNAMIC_EARLY_SIZE` versus `kmem_cache_cpu` on non-4K pages
    (fixed by raising `PERCPU_DYNAMIC_SIZE_SHIFT` under `CONFIG_DEPT`)
  Review size/offset `BUILD_BUG_ON`s, fixed reserves, and `struct page`
  layout assumptions with DEPT in mind.
- Static lock initializers must initialize the DEPT map:
  `STATIC_LOCKDEP_MAP_INIT()`, `__DEP_MAP_MUTEX_INITIALIZER()`,
  `__PERCPU_RWSEM_DEP_MAP_INIT()`, `SEQCOUNT_DEP_MAP_INIT()`,
  `__SRCU_DEP_MAP_INIT()` all set `.dmap = DEPT_MAP_INITIALIZER(...)`. A
  new lock type with its own static initializer needs the same.
- `include/linux/dept.h` is included from `irqflags.h`, `lockdep_types.h`
  and `mm_types.h`. It must stay free of includes and depend only on
  forward declarations.

## Judging Patches That Respond to a DEPT Report

DEPT is EXPERIMENTAL and produces false positives. Changing kernel locking
to fix a false positive adds risk for no gain. Treating a real report as
noise ships a deadlock.

**Reading a report.** Rewrite each context of the summary as one edge: in
context X, `[S]` class P, then `[W]` class Q, gives `P → Q`, because `[E]`
of P cannot happen while waiting for Q. The report is the cycle of these
edges.

**Before calling it real**, check every node and edge:
1. **Class origin.** `&obj->lock:N` is a named map. `<sched>` is a
   callsite key from an unmapped staged wait. `(unknown)` means the start
   is only a timestamp. For each `<sched>` or callsite class, open the
   `[W]` function and ask whether semantically different channels pass
   through that line. If they do, the cycle may come from merged classes.
2. **Attribution.** For `[E] try_to_wake_up(<sched>)`, check that the
   waker really is the producer of the wait it woke, and not a signal or an
   unrelated `wake_up_process()`.
3. **Coexistence.** Boot-only code (initcalls), single-threaded phases,
   one-shot objects, and different context ids cannot form a cycle.
4. **Semantics DEPT does not model.** rwsem read-read, timeout waits,
   distinct folio instances, and `lock_cmp_fn` orderings that are not the
   innermost.
5. **Graph evidence.** `/proc/dept_deps` lists every class edge. A class
   with edges from unrelated subsystems is an over-merged class.

**Real deadlock fixes** usually remove one edge without adding a lock:
- move the blocking operation out of the held context (ntfs3: call
  `truncate_setsize()` before `ni_lock()`)
- replace the inner blocking acquire with a trylock and retry after
  dropping the outer lock
- remove the need to hold the inner object at all (hugetlb: answer the
  question with `folio_test_anon()` instead of locking the pagecache
  folio)
Apply the normal locking review (`locking.md`) to the new ordering and
check every caller of the moved code.

**False positive fixes** change granularity or annotation strength, never
locking:
- split a merged class by moving the key to where the channels differ
  (macro-level `static struct dept_key`, or a key in the per-channel
  structure, passed down through `sdt_map_init_key()` and
  `init_completion_dmap()`)
- separate instances of one channel with an init-time subclass
  (`dept_map_init(m, k, sub_u, n)`, `sub_u < DEPT_MAX_SUBCLASSES_USR`)
- narrow an annotation that is too strong (trylock form for assert-only
  acquires; explicit `sdt_ecxt_enter()`/`sdt_ecxt_exit()` to bound an event
  context)
- as a last resort, mark the map `nocheck`, with the reason in the
  changelog

The changelog should include the report, the two call chains that
collided, and the granularity or annotation choice. It should also say how
the patch was verified: the report is gone *and* known deadlocks are still
detected (`CONFIG_DEPT_UNIT_TEST`, the lkdtm `FOLIO_LOCK_AA`,
`FOLIO_LOCK_ABBA` and `FOLIO_MUTEX_LOCK_ABBA` scenarios, and
`tools/testing/selftests/rdtm/`).

**REPORT as bugs**:
- A patch that changes lock ordering, adds locks, or restructures
  non-DEPT code to silence a report whose cycle depends on a `<sched>` or
  callsite class, on rwsem read-read, or on two folio instances, without
  showing that the merged waits are the same channel.
- A patch that silences a report with `lockdep_set_novalidate_class()`,
  `nocheck`, `dept_off()`, a timeout conversion, or by removing a
  `dept_page_wait_on_bit()`/`sdt_*` annotation, without explaining why the
  cycle is impossible.
- A granularity change to per-instance keys (`NULL` key) for short-lived
  objects. It makes DEPT blind for that channel.
- A false-positive suppression in the engine that is not limited to the
  analysed case. Over-broad suppression has hidden real deadlocks before.
  In the hugetlb thread, DEPT printed the ABBA only after a report
  de-duplication flag (`tc->reported`) was removed.

## Quick Checks

- **`!CONFIG_DEPT` stubs**: every new API in `dept.h`, `dept_sdt.h` or
  `dept_ldt.h` needs a stub that still evaluates or consumes arguments
  used only by DEPT (`(void)(k)`, `(void)(n)`). Otherwise
  `static struct dept_key` and name strings trigger unused warnings.
- **Exports**: a DEPT call added to a header macro or inline that modules
  use (`local_irq_*()`, `wait_*()`, `folio_lock()`) needs
  `EXPORT_SYMBOL_GPL` on the engine function, as was done for
  `dept_hardirqs_off()`.
- **Rust helpers**: a C helper in `rust/helpers/` that expands an annotated
  macro (`rust_helper_wait_for_completion()`) is one expansion site. All
  Rust callers share its class.
- **Name lifetime**: class names point at the string passed at map init.
  Names in freed module memory are covered only by `dept_free_range()`.
  Names in other dynamic memory are not.
- **One event bit per event**: `dept_event()`, `dept_ecxt_enter()` and
  `dept_map_ecxt_modify()` warn when `e_f` has more than one bit set and
  use only the first.
- **Wait-before-request ordering**: `sdt_wait()` calls
  `dept_request_event()` before `dept_wait()`. Code that waits and then
  takes ownership must re-arm the request after the wait (see the
  page-flag hand-off rule).
- **Exit with an event context held**: `dept_task_exit()` drops all
  held ecxts and history. A task that exits with an ecxt held (lock leaked
  across exit) loses that information without a DEPT warning.
