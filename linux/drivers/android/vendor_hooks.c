// SPDX-License-Identifier: GPL-2.0-only
/* vendor_hook.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#define CREATE_TRACE_POINTS
#include <trace/hooks/vendor_hooks.h>
#include <linux/tracepoint.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/fpsimd.h>
#include <trace/hooks/binder.h>
#include <trace/hooks/futex.h>
#include <trace/hooks/dtask.h>
#include <trace/hooks/cpuidle.h>
#include <trace/hooks/topology.h>
#include <trace/hooks/mpam.h>
#include <trace/hooks/gic.h>
#include <trace/hooks/wqlockup.h>
#include <trace/hooks/debug.h>
#include <trace/hooks/sysrqcrash.h>
#include <trace/hooks/printk.h>
#include <trace/hooks/gic_v3.h>
#include <trace/hooks/epoch.h>
#include <trace/hooks/cpufreq.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/preemptirq.h>
#include <trace/hooks/ftrace_dump.h>
#include <trace/hooks/ufshcd.h>
#include <trace/hooks/cgroup.h>
#include <trace/hooks/sys.h>
#include <trace/hooks/iommu.h>
#include <trace/hooks/net.h>
#include <trace/hooks/timer.h>
#include <trace/hooks/pm_domain.h>
#include <trace/hooks/cpuidle_psci.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/avc.h>
#include <trace/hooks/creds.h>
#include <trace/hooks/memory.h>
#include <trace/hooks/module.h>
#include <trace/hooks/selinux.h>
#include <trace/hooks/syscall_check.h>
#include <trace/hooks/logbuf.h>
#include <trace/hooks/remoteproc.h>
#include <trace/hooks/hung_task.h>
#include <trace/hooks/bug.h>
#include <trace/hooks/softlockup.h>
#include <trace/hooks/power.h>
#include <trace/hooks/fault.h>
#include <trace/hooks/traps.h>
#include <trace/hooks/fips140.h>
#include <trace/hooks/thermal.h>
#include <trace/hooks/rwsem.h>
#include <trace/hooks/timekeeping.h>
#include <trace/hooks/audio_usboffload.h>
#include <trace/hooks/drm_framebuffer.h>
#include <trace/hooks/drm_atomic.h>
#include <trace/hooks/psci.h>
#include <trace/hooks/usb.h>
#include <trace/hooks/regmap.h>
#include <trace/hooks/dmabuf.h>
#include <trace/hooks/mmc.h>

/*
 * Export tracepoints that act as a bare tracehook (ie: have no trace event
 * associated with them) to allow external modules to probe them.
 */
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_select_task_rq_fair);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_select_task_rq_rt);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_select_task_rq_dl);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_select_fallback_rq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_scheduler_tick);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_enqueue_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_dequeue_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_can_migrate_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_find_lowest_rq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_rtmutex_prepare_setprio);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_prepare_prio_fork);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_finish_prio_fork);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_user_nice);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_setscheduler);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_arch_set_freq_scale);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_is_fpsimd_save);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_binder_transaction_init);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_binder_set_priority);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_binder_restore_priority);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_binder_wakeup_ilocked);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_alter_futex_plist_add);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_mutex_wait_start);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_mutex_wait_finish);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rtmutex_wait_start);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rtmutex_wait_finish);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rwsem_read_wait_start);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rwsem_read_wait_finish);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rwsem_write_wait_start);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rwsem_write_wait_finish);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_sched_show_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpu_idle_enter);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpu_idle_exit);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_mpam_set);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_find_busiest_group);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_gic_resume);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_wq_lockup_pool);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ipi_stop);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_sysrq_crash);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_dump_throttled_rt_tasks);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_printk_hotplug);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_printk_caller_id);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_printk_caller);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_printk_ext_header);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_jiffies_update);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_gic_v3_set_affinity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_gic_set_affinity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_gic_v3_affinity_init);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_show_suspend_epoch_val);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_show_resume_epoch_val);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_freq_table_limits);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpufreq_resolve_freq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpufreq_fast_switch);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpufreq_target);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_newidle_balance);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_nohz_balancer_kick);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_rebalance_domains);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_find_busiest_queue);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_migrate_queued_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_find_energy_efficient_cpu);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_set_sugov_sched_attr);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_iowait);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_sugov_update);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cpu_overutilized);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_setaffinity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_getaffinity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_update_cpus_allowed);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_skip_swapcache_flags);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_gfp_zone_flags);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_readahead_gfp_mask);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_preempt_disable);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_preempt_enable);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_irqs_disable);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_irqs_enable);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_task_cpu);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_try_to_wake_up);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_try_to_wake_up_success);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_fork);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_wake_up_new_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_new_task_stats);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_flush_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_tick_entry);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_schedule);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_cpu_starting);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_cpu_dying);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_account_irq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_account_irq_start);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_account_irq_end);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_place_entity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_build_perf_domains);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_update_cpu_capacity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_update_misfit_status);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cpu_cgroup_attach);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cpu_cgroup_can_attach);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cpu_cgroup_online);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_fork_init);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_ttwu_cond);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_schedule_bug);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_exec);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_map_util_freq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_em_cpu_energy);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ftrace_oops_enter);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ftrace_oops_exit);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ftrace_size_check);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ftrace_format_check);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ftrace_dump_buffer);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_fill_prdt);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_prepare_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_update_sysfs);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_send_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_compl_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cgroup_set_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_syscall_prctl_finished);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_send_uic_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_send_tm_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_check_int_errors);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_update_sdev);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cgroup_attach);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_iommu_setup_dma_ops);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_iommu_iovad_alloc_iova);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_iommu_iovad_free_iova);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ptype_head);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_kfree_skb);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_timer_calc_index);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_allow_domain_state);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpuidle_psci_enter);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpuidle_psci_exit);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cgroup_force_kthread_migration);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_check_preempt_tick);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_check_preempt_wakeup_ignore);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_replace_next_task_fair);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_do_sched_yield);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_binder_wait_for_work);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_sync_txn_recvd);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_build_sched_domains);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_update_topology_flags_workfn);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_sched_balance_rt);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_pick_next_entity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_check_preempt_wakeup);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cpufreq_transition);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_balance_anon_file_reclaim);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_show_max_freq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_free_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_after_enqueue_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_after_dequeue_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_enqueue_entity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_dequeue_entity);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_entity_tick);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_enqueue_task_fair);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_dequeue_task_fair);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_selinux_avc_insert);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_selinux_avc_node_delete);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_selinux_avc_node_replace);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_selinux_avc_lookup);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_commit_creds);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_exit_creds);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_override_creds);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_revert_creds);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_set_memory_nx);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_set_memory_rw);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_set_module_permit_before_init);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_set_module_permit_after_init);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_oom_check_panic);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_selinux_is_initialized);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_check_mmap_file);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_check_file_open);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_check_bpf_syscall);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_cpus_allowed_ptr_locked);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_rto_next_cpu);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_is_cpu_allowed);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_get_nohz_timer_target);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_em_dev_register_pd);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_logbuf);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_logbuf_pr_cont);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_post_init_entity_util_avg);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_find_new_ilb);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rproc_recovery);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_check_uninterruptible_tasks);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_check_uninterruptible_tasks_dn);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_uclamp_eff_get);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_util_est_update);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_setscheduler_uclamp);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_do_wake_up_sync);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_set_wake_flags);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_effective_cpu_util);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_report_bug);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_watchdog_timer_softlockup);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_try_to_freeze_todo);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_try_to_freeze_todo_unfrozen);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_die_kernel_fault);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_do_sea);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_do_mem_abort);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_do_sp_pc_abort);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_do_undefinstr);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_do_ptrauth_fault);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_panic_unhandled);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_arm64_serror_panic);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_sha256);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_aes_expandkey);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_aes_encrypt);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_aes_decrypt);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_modify_thermal_request_freq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_modify_thermal_target_freq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_thermal_register);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_thermal_unregister);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rproc_recovery_set);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_enable_thermal_power_throttle);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rwsem_init);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rwsem_wake);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_rwsem_write_finished);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_alter_rwsem_list_add);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_thermal_power_cap);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_tk_based_time_sync);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_kswapd_per_node);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_audio_usb_offload_vendor_set);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_audio_usb_offload_ep_action);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_audio_usb_offload_synctype);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_audio_usb_offload_connect);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_audio_usb_offload_disconnect);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_atomic_remove_fb);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_drm_atomic_check_modeset);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_get_thermal_zone_device);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_psci_tos_resident_on);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_psci_cpu_suspend);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_usb_new_device_added);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_force_compatible_pre);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_force_compatible_post);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_regmap_update);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_dma_buf_release);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_mmc_check_status);
