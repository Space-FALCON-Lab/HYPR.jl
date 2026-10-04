"""Complete source inventory and Julia line-coverage gate, with no exclusions."""
from pathlib import Path
import hashlib, json, re, sys, tomllib
MIN_OVERALL, MIN_FILE = 90.0, 80.0

def check(root, manifest):
    root=Path(root); sources=manifest["sources"]
    actual={str(p.relative_to(root)) for d in ("src","ext") for p in (root/d).rglob("*.jl")}
    failures=[];rows=[]
    if actual != set(sources):
        failures.append("Source inventory mismatch")
    for rel in sorted(actual):
        p=root/rel; source=p.read_text(); record=sources.get(rel,{})
        if hashlib.sha256(p.read_bytes()).hexdigest()!=record.get("sha256"):
            failures.append("Source identity mismatch: "+rel)
        lines=source.splitlines(); counts={}; artifacts=list(p.parent.glob(p.name+".*.cov"))
        for artifact in artifacts:
            coverage=artifact.read_text().splitlines()
            if len(coverage)!=len(lines):
                failures.append("Truncated coverage: "+str(artifact));continue
            for i,(cov,original) in enumerate(zip(coverage,lines),1):
                match=re.fullmatch(r"\s*([0-9]+|-) (.*)",cov)
                if not match or match[2]!=original:
                    failures.append("Coverage/source mismatch: "+str(artifact)+":"+str(i));continue
                if match[1]!="-":counts[i]=counts.get(i,0)+int(match[1])
        # Julia may omit counters for wholly uncompiled methods. Refuse that
        # incomplete measurement rather than reporting an inflated percentage.
        for span in record.get("functions",[]):
            if not any(i in counts for i in range(span["first"],span["last"]+1)):
                failures.append("Unmeasured function: "+rel+":"+str(span["first"])+" "+span["label"])
        if not counts and not record.get("module_wiring_only",False):
            failures.append("Missing executable coverage: "+rel)
        covered=sum(v>0 for v in counts.values());total=len(counts)
        pct=100*covered/total if total else None
        if pct is not None and pct<MIN_FILE:failures.append("Below per-file threshold: "+rel)
        rows.append(dict(path=rel,covered=covered,executable=total,percent=pct,
                         module_wiring_only=record.get("module_wiring_only",False),artifacts=len(artifacts)))
    covered=sum(row["covered"] for row in rows);total=sum(row["executable"] for row in rows)
    pct=100*covered/total if total else 0
    if pct<MIN_OVERALL:failures.append("Below overall threshold")
    return dict(passed=not failures,covered=covered,executable=total,percent=pct,
        thresholds=dict(overall=MIN_OVERALL,per_file=MIN_FILE),files=rows,failures=failures)

def main():
    root=Path(__file__).resolve().parent.parent
    if len(sys.argv)!=3:raise SystemExit("usage: coverage_gate.py inventory.toml report.json")
    result=check(root,tomllib.loads(Path(sys.argv[1]).read_text()))
    Path(sys.argv[2]).write_text(json.dumps(result,indent=2)+"\n")
    for row in result["files"]:print(row["path"],row["covered"],row["executable"],row["percent"])
    print("OVERALL",result["percent"],"FAILURES",result["failures"])
    raise SystemExit(0 if result["passed"] else 1)
if __name__=="__main__":main()
