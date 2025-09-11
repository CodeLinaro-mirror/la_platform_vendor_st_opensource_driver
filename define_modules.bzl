load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files", "strip_prefix")

def define_modules(target, variant):
    tv = "{}_{}".format(target, variant)

    copts = []
    deps = select({
        "//build/kernel/kleaf:socrepo_true": [
            "//soc-repo:all_headers",
            "//soc-repo:{}/drivers/pinctrl/qcom/pinctrl-msm".format(tv),
        ],
        "//build/kernel/kleaf:socrepo_false": ["//msm-kernel:all_headers"],
    })
    kernel_build = select({
        "//build/kernel/kleaf:socrepo_true": "//soc-repo:{}_base_kernel".format(tv),
        "//build/kernel/kleaf:socrepo_false": "//msm-kernel:{}".format(tv),
    })
    if target == "sun":
        copts.append("-DNFC_SECURE_PERIPHERAL_ENABLED")
        deps += [
            "//vendor/qcom/opensource/securemsm-kernel:smcinvoke_kernel_headers",
            "//vendor/qcom/opensource/securemsm-kernel:{}_smcinvoke_dlkm".format(tv),
        ]

    if target == "canoe":
        copts.append("-DCONFIG_NFC_BOB1")
        copts.append("-DNFC_SECURE_PERIPHERAL_ENABLED")
        deps += [
            "//vendor/qcom/opensource/securemsm-kernel:smcinvoke_kernel_headers",
            "//vendor/qcom/opensource/securemsm-kernel:{}_smcinvoke_dlkm".format(tv),
        ]
    if target == "art":
        copts.append("-DCONFIG_NFC_BOB1")
        #copts.append("-DNFC_SECURE_PERIPHERAL_ENABLED")
        #deps += [
        #    "//vendor/qcom/opensource/securemsm-kernel:smcinvoke_kernel_headers",
        #    "//vendor/qcom/opensource/securemsm-kernel:{}_smcinvoke_dlkm".format(tv),
        #]

    ddk_module(
        name = "{}_stm_nfc_i2c".format(tv),
        out = "stm_nfc_i2c.ko",
        srcs = [
            "nfc/st21nfc.c",
            "nfc/st21nfc.h",
        ],
        hdrs = ["include/uapi/linux/nfc/st_uapi.h"],
        includes = [".", "linux", "nfc", "include/uapi/linux/nfc"],
        copts = copts,
        deps = deps,
        kernel_build = kernel_build,
        visibility = ["//visibility:public"],
    )

    pkg_files(
        name = tv + "_dist_files",
        srcs = [":{}_stm_nfc_i2c".format(tv)],
        visibility = ["//visibility:private"],
        strip_prefix = strip_prefix.files_only(),
    )

    pkg_install(
        name = "{}_stm_nfc_i2c_dist".format(tv),
        srcs = [":{}_dist_files".format(tv)],
        destdir = "out/target/product/{}/dlkm/lib/modules/".format(target),
    )
