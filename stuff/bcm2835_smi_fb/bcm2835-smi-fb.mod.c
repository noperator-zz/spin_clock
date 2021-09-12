#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
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
__used
__attribute__((section("__versions"))) = {
	{ 0x57d780b1, "module_layout" },
	{ 0x7eebba84, "cdev_del" },
	{ 0x1dfc4c8b, "cdev_init" },
	{ 0x2e5810c6, "__aeabi_unwind_cpp_pr1" },
	{ 0xb0b9f3f7, "of_parse_phandle" },
	{ 0xab7a4d1d, "bcm2835_smi_set_regs_from_settings" },
	{ 0x74f23ae8, "device_destroy" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
	{ 0x28cc25db, "arm_copy_from_user" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x72db83c4, "bcm2835_smi_set_address" },
	{ 0xa669e8a6, "__platform_driver_register" },
	{ 0xf4fa543b, "arm_copy_to_user" },
	{ 0xf6720336, "bcm2835_smi_get_settings_from_regs" },
	{ 0x5f754e5a, "memset" },
	{ 0x8cadeebc, "bcm2835_smi_get" },
	{ 0x846278b4, "device_create" },
	{ 0xf1395bb, "_dev_err" },
	{ 0xa00dc18f, "bcm2835_smi_write_buf" },
	{ 0xd1929a, "cdev_add" },
	{ 0x8be5b384, "_dev_info" },
	{ 0xdb7305a1, "__stack_chk_fail" },
	{ 0x3e0ccb4f, "bcm2835_smi_user_dma" },
	{ 0x5a106718, "class_destroy" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0x6a04b677, "platform_driver_unregister" },
	{ 0x2a4f9b72, "devm_kmalloc" },
	{ 0x421b1b55, "__class_create" },
	{ 0xb33bd6cb, "bcm2835_smi_read_buf" },
	{ 0xc57877cd, "down_timeout" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=bcm2835_smi";

MODULE_ALIAS("of:N*T*Cbrcm,bcm2835-smi-fb");
MODULE_ALIAS("of:N*T*Cbrcm,bcm2835-smi-fbC*");

MODULE_INFO(srcversion, "E7B7AA5CEC47CBA4D63F739");
