"""Each incorrect kernel must fail an arithmetic assertion in a fresh process."""
import os, pathlib, subprocess, sys
root = pathlib.Path(__file__).resolve().parent
julia = os.environ.get("JULIA", "julia")
project = os.environ.get("HYPR_TEST_PROJECT", str(root.parent))
flags = ["--startup-file=no", "--compiled-modules=existing", "--pkgimages=existing"]
names = ["none", "eval_order", "cull_before_stop", "no_final_move", "fp_order", "rpo_draw_swap", "rpo_fp_order"]
for name in names:
    result = subprocess.run([julia, *flags, "--project="+project, str(root/"run_arithmetic_case.jl"), name], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print("MUTANT", name, "EXIT", result.returncode, flush=True)
    if name == "none":
        ok = result.returncode == 0
    else:
        ok = result.returncode != 0 and "Test Failed" in result.stdout and "MUTANT_APPLIED="+name in result.stdout
    if not ok:
        print(result.stdout)
        raise SystemExit("Missing arithmetic discrimination: "+name)
    if name != "none":
        print("\n".join(line for line in result.stdout.splitlines() if "Test Failed" in line or "Test Summary" in line or "Some tests did not pass" in line))
print("All six deliberately incorrect kernels rejected; unchanged control passes.")
