#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xc8d01d53, "module_layout" },
	{ 0x603166e8, "cdev_alloc" },
	{ 0x37ce6741, "cdev_del" },
	{ 0xde310d05, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x51b1c11d, "cdev_init" },
	{ 0xf9a482f9, "msleep" },
	{ 0x426b20ed, "put_devmap_managed_page" },
	{ 0xd6ee688f, "vmalloc" },
	{ 0x3fd78f3b, "register_chrdev_region" },
	{ 0xc3690fc, "_raw_spin_lock_bh" },
	{ 0xd6328bc3, "skb_copy" },
	{ 0xa426ca69, "device_destroy" },
	{ 0xbbfd9481, "alloc_pages" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x999e8297, "vfree" },
	{ 0x97651e6c, "vmemmap_base" },
	{ 0x71038ac7, "pv_ops" },
	{ 0x851c15cf, "kthread_create_on_node" },
	{ 0x3363cbcd, "kthread_bind" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x356461c8, "rtc_time64_to_tm" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xfb578fc5, "memset" },
	{ 0xd38cd261, "__default_kernel_pte_mask" },
	{ 0x4c9f47a5, "current_task" },
	{ 0xa0aa95d4, "kthread_stop" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0xb5b3b8de, "vmap" },
	{ 0x3ebaa824, "kfree_skb_reason" },
	{ 0xf1969a8e, "__usecs_to_jiffies" },
	{ 0xa7d3d17b, "device_create" },
	{ 0xa71ebc8a, "init_net" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0xdd558c3b, "nf_register_net_hook" },
	{ 0x83ff2ea1, "vm_insert_page" },
	{ 0xfe5d4bb2, "sys_tz" },
	{ 0xef597490, "nf_unregister_net_hook" },
	{ 0xabac4112, "cdev_add" },
	{ 0x800473f, "__cond_resched" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0xff21ae65, "__free_pages" },
	{ 0x6383b27c, "__x86_indirect_thunk_rdx" },
	{ 0xe46021ca, "_raw_spin_unlock_bh" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x1000e51, "schedule" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x92997ed8, "_printk" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xce276787, "wake_up_process" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0xaf88e69b, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x37a0cba, "kfree" },
	{ 0x94961283, "vunmap" },
	{ 0x69acdf38, "memcpy" },
	{ 0x3ee2b36d, "class_destroy" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x1e5e60ca, "skb_copy_bits" },
	{ 0x9b0fb107, "__class_create" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xeeb1a228, "__put_page" },
	{ 0xec925c6d, "get_user_pages_fast" },
	{ 0x587f22d7, "devmap_managed_key" },
	{ 0x8a35b432, "sme_me_mask" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "0884B47503C51CD3B6562E1");
