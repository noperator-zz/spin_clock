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
	{ 0x6a04b677, "platform_driver_unregister" },
	{ 0xa669e8a6, "__platform_driver_register" },
	{ 0x8c03d20c, "destroy_workqueue" },
	{ 0x42160169, "flush_workqueue" },
	{ 0xfe990052, "gpio_free" },
	{ 0xc1514a3b, "free_irq" },
	{ 0xc57877cd, "down_timeout" },
	{ 0x28cc25db, "arm_copy_from_user" },
	{ 0x3e0ccb4f, "bcm2835_smi_user_dma" },
	{ 0x808b71a2, "gpiod_get_raw_value" },
	{ 0x1309a06a, "framebuffer_release" },
	{ 0xcd3b2b0f, "unregister_framebuffer" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0xacc4569a, "gpiod_to_irq" },
	{ 0xa4002818, "gpiod_direction_input" },
	{ 0xa969852b, "gpiod_set_raw_value" },
	{ 0xf9a482f9, "msleep" },
	{ 0xed94b4a6, "gpiod_direction_output_raw" },
	{ 0x73a0bff8, "gpio_to_desc" },
	{ 0x47229b5c, "gpio_request" },
	{ 0x43a53735, "__alloc_workqueue_key" },
	{ 0x5a91adbf, "register_framebuffer" },
	{ 0xc631580a, "console_unlock" },
	{ 0x26dbc906, "fb_set_var" },
	{ 0xfbaaf01e, "console_lock" },
	{ 0xee58e970, "fb_add_videomode" },
	{ 0x96c17136, "fb_var_to_videomode" },
	{ 0x91715312, "sprintf" },
	{ 0xef5a6b3b, "framebuffer_alloc" },
	{ 0xab7a4d1d, "bcm2835_smi_set_regs_from_settings" },
	{ 0x8cadeebc, "bcm2835_smi_get" },
	{ 0xb0b9f3f7, "of_parse_phandle" },
	{ 0xf1395bb, "_dev_err" },
	{ 0xb2d48a2e, "queue_work_on" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0xdb7305a1, "__stack_chk_fail" },
	{ 0x5f754e5a, "memset" },
	{ 0x86445f46, "dma_alloc_from_dev_coherent" },
	{ 0x8c2ca5cf, "remap_pfn_range" },
	{ 0xee3df27e, "arm_dma_ops" },
	{ 0x16305289, "warn_slowpath_null" },
	{ 0x818c415d, "dma_release_from_dev_coherent" },
	{ 0x2e5810c6, "__aeabi_unwind_cpp_pr1" },
	{ 0x7c32d0f0, "printk" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=bcm2835_smi";

MODULE_ALIAS("of:N*T*Cbrcm,bcm2835-smi-fb");
MODULE_ALIAS("of:N*T*Cbrcm,bcm2835-smi-fbC*");

MODULE_INFO(srcversion, "1AF9FE1C5B541E2217FC851");
