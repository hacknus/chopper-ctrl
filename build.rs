use std::process::Command;

fn main() {
    let output = Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .unwrap();
    let git_hash = String::from_utf8(output.stdout).unwrap();
    println!("cargo:rustc-env=GIT_HASH={}", git_hash);

    let output = Command::new("git")
        .args(["rev-parse", "--abbrev-ref", "HEAD"])
        .output()
        .unwrap();
    let git_branch = String::from_utf8(output.stdout).unwrap();
    println!("cargo:rustc-env=GIT_BRANCH={}", git_branch);

    let mut b = freertos_cargo_build::Builder::new();

    // Path to FreeRTOS kernel or set ENV "FREERTOS_SRC" instead
    b.freertos("FreeRTOS-Kernel");
    b.freertos_config("include"); // Location of `FreeRTOSConfig.h`
    b.freertos_port("GCC/ARM_CM4F".to_string()); // Port dir relative to 'FreeRTOS-Kernel/portable'
    // b.heap("heap4.c".to_string());
    // Set the heap_?.c allocator to use from
    // 'FreeRTOS-Kernel/portable/MemMang' (Default: heap_4.c)

    // b.get_cc().file("More.c");   // Optional additional C-Code to be compiled

    b.compile().unwrap_or_else(|e| panic!("{}", e.to_string()));
}
