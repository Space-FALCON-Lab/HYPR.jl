"""Run the full isolated measurement. Never accepts stale or failed stages."""
import argparse, json, os, pathlib, shutil, subprocess

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--core-project",required=True)
    p.add_argument("--integration-project",required=True)
    p.add_argument("--output",required=True)
    args=p.parse_args()
    root=pathlib.Path(__file__).resolve().parent.parent
    out=pathlib.Path(args.output).resolve()
    if out.exists() and any(out.iterdir()):
        raise SystemExit("Coverage output must be empty; retain earlier results separately.")
    out.mkdir(parents=True,exist_ok=True)
    core_project=str(pathlib.Path(args.core_project).resolve())
    integration_project=str(pathlib.Path(args.integration_project).resolve())
    julia=os.environ.get("JULIA","julia")
    for directory in (root/"src",root/"ext"):
        for f in directory.rglob("*.cov"): f.unlink()
    flags=["--startup-file=no","--compiled-modules=existing","--pkgimages=existing"]
    env={**os.environ,"JULIA_LOAD_PATH":"@:@stdlib","JULIA_PKG_PRECOMPILE_AUTO":"0","OPENBLAS_NUM_THREADS":"1"}
    stages=[]
    commands=[("inventory",[julia,*flags,str(root/"scripts/coverage_inventory.jl"),str(out/"sources.toml")],1)]
    for threads in (1,4):
        commands.append(("core_t"+str(threads),[julia,*flags,"--check-bounds=yes","--code-coverage=@"+str(root),"--project="+core_project,str(root/"test/runtests.jl")],threads))
    commands.append(("extension",[julia,*flags,"--code-coverage=@"+str(root),"--project="+integration_project,str(root/"test/extension_coverage.jl")],4))
    # The callback contract uses a test-only observer method in a fresh process.
    commands.append(("rrt_callback_contract",[julia,*flags,"--code-coverage=@"+str(root),"--project="+integration_project,str(root/"test/rrt_callback_contract.jl")],1))
    import sys
    commands.append(("gate",[sys.executable,str(root/"scripts/coverage_gate.py"),str(out/"sources.toml"),str(out/"report.json")],1))
    for name,cmd,threads in commands:
        if name=="gate":
            for directory in (root/"src",root/"ext"):
                for artifact in directory.rglob("*.cov"):
                    dest=out/"raw"/artifact.relative_to(root)
                    dest.parent.mkdir(parents=True,exist_ok=True)
                    shutil.copyfile(artifact,dest)
        env["JULIA_NUM_THREADS"]=str(threads)
        with (out/(name+".log")).open("w") as log:
            code=subprocess.run(cmd,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT).returncode
        stages.append(dict(stage=name,exit_code=code))
        (out/"stages.json").write_text(json.dumps(stages,indent=2)+"\n")
        print(name,"exit",code,flush=True)
        if code:raise SystemExit(code)
    print("HYPR coverage run and gate passed",flush=True)
if __name__=="__main__":main()
