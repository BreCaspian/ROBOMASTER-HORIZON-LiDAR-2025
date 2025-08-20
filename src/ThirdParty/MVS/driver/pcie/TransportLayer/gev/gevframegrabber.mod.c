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
	{ 0x2f2c95c4, "flush_work" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x7a3848cc, "dma_map_sg_attrs" },
	{ 0x37ce6741, "cdev_del" },
	{ 0xde310d05, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x51b1c11d, "cdev_init" },
	{ 0xf9a482f9, "msleep" },
	{ 0x426b20ed, "put_devmap_managed_page" },
	{ 0x2caa7bd4, "pci_free_irq_vectors" },
	{ 0xb84e5552, "kernel_sendmsg" },
	{ 0xd6ee688f, "vmalloc" },
	{ 0x754d539c, "strlen" },
	{ 0xcc2dfcaf, "dma_unmap_sg_attrs" },
	{ 0x69449ab2, "pci_disable_device" },
	{ 0x3f170407, "sock_release" },
	{ 0xc3690fc, "_raw_spin_lock_bh" },
	{ 0xa426ca69, "device_destroy" },
	{ 0x87b8798d, "sg_next" },
	{ 0xbbfd9481, "alloc_pages" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xa0bcb084, "pci_release_regions" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x136162fb, "sock_create_kern" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x999e8297, "vfree" },
	{ 0x46afdcba, "dma_free_attrs" },
	{ 0x97651e6c, "vmemmap_base" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x71038ac7, "pv_ops" },
	{ 0x851c15cf, "kthread_create_on_node" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x3363cbcd, "kthread_bind" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x356461c8, "rtc_time64_to_tm" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xaff2dd5d, "pci_set_master" },
	{ 0xf7242b1e, "pci_alloc_irq_vectors_affinity" },
	{ 0xfb578fc5, "memset" },
	{ 0xd38cd261, "__default_kernel_pte_mask" },
	{ 0x4c9f47a5, "current_task" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xa0aa95d4, "kthread_stop" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0xde80cd09, "ioremap" },
	{ 0xb5b3b8de, "vmap" },
	{ 0x9d2ab8ac, "__tasklet_schedule" },
	{ 0x715a5ed0, "vprintk" },
	{ 0xce11de54, "dma_alloc_attrs" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xf1969a8e, "__usecs_to_jiffies" },
	{ 0x2364c85a, "tasklet_init" },
	{ 0xa7d3d17b, "device_create" },
	{ 0x7d6dad01, "platform_device_unregister" },
	{ 0x24d273d1, "add_timer" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xa71ebc8a, "init_net" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0xfe5d4bb2, "sys_tz" },
	{ 0xea3c74e, "tasklet_kill" },
	{ 0xabac4112, "cdev_add" },
	{ 0x800473f, "__cond_resched" },
	{ 0x3a2f6702, "sg_alloc_table" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0x9f984513, "strrchr" },
	{ 0xff21ae65, "__free_pages" },
	{ 0xd7aaf040, "platform_device_register" },
	{ 0xe46021ca, "_raw_spin_unlock_bh" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x1000e51, "schedule" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x92997ed8, "_printk" },
	{ 0xead99a33, "pci_read_config_dword" },
	{ 0xce276787, "wake_up_process" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x1c937e3f, "pci_unregister_driver" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0xaf88e69b, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x148653, "vsnprintf" },
	{ 0x640663dd, "pci_irq_vector" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x82ee90dc, "timer_delete_sync" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x37a0cba, "kfree" },
	{ 0x94961283, "vunmap" },
	{ 0x69acdf38, "memcpy" },
	{ 0xe3f4c62c, "pci_request_regions" },
	{ 0x43ea87ab, "pci_disable_msi" },
	{ 0xedc03953, "iounmap" },
	{ 0x9c1ba16a, "kernel_bind" },
	{ 0x2554c13b, "__pci_register_driver" },
	{ 0x3ee2b36d, "class_destroy" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x7f5b4fe4, "sg_free_table" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x4a453f53, "iowrite32" },
	{ 0xa74ccb9b, "pci_enable_device" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x9b0fb107, "__class_create" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xeeb1a228, "__put_page" },
	{ 0xa78af5f3, "ioread32" },
	{ 0xec925c6d, "get_user_pages_fast" },
	{ 0xc1514a3b, "free_irq" },
	{ 0xe914e41e, "strcpy" },
	{ 0x587f22d7, "devmap_managed_key" },
	{ 0x8a35b432, "sme_me_mask" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v000010EEd00007024sv000010EEsd00002147bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00007028sv000010EEsd00002647bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00007028sv000010EEsd00002547bc*sc*i*");

MODULE_INFO(srcversion, "A929404C8EB0447CF45100C");
