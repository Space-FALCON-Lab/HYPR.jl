import hashlib, importlib.util, json, os, pathlib, shutil, subprocess, sys, tempfile, unittest
spec=importlib.util.spec_from_file_location("gate",pathlib.Path(__file__).resolve().parents[1]/"scripts/coverage_gate.py")
gate=importlib.util.module_from_spec(spec);spec.loader.exec_module(gate)
class GateTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=pathlib.Path(self.tmp.name)
        (self.root/"src").mkdir();(self.root/"ext").mkdir()
        self.manifest={"sources":{}}
        self.add("src/core.jl",10,10);self.add("ext/adapter.jl",10,10)
    def tearDown(self):self.tmp.cleanup()
    def add(self,name,total,covered):
        source="".join("value_"+str(i)+" = "+str(i)+"\n" for i in range(total))
        p=self.root/name;p.write_text(source)
        self.manifest["sources"][name]=dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),functions=[dict(first=1,last=total,label="fixture")],module_wiring_only=False)
        (p.parent/(p.name+".1.cov")).write_text("".join(f"{int(i<covered):9d} "+line for i,line in enumerate(source.splitlines(keepends=True))))
    def result(self):return gate.check(self.root,self.manifest)
    def test_complete_positive(self):self.assertTrue(self.result()["passed"])
    def test_exact_thresholds_pass(self):
        self.add("ext/adapter.jl",10,8);self.assertTrue(self.result()["passed"])
    def test_per_file_failure_even_with_high_overall(self):
        self.add("src/core.jl",100,100);self.add("ext/adapter.jl",10,7);self.assertFalse(self.result()["passed"])
    def test_overall_failure_even_with_files_above_floor(self):
        self.add("src/core.jl",10,8);self.add("ext/adapter.jl",10,8);self.assertFalse(self.result()["passed"])
    def test_missing_extension_file(self):
        (self.root/"ext/adapter.jl.1.cov").unlink();self.assertFalse(self.result()["passed"])
    def test_inventory_omission(self):
        self.manifest["sources"].pop("ext/adapter.jl");self.assertFalse(self.result()["passed"])
    def test_new_source_not_in_inventory(self):
        (self.root/"ext/new.jl").write_text("f()=1\n");self.assertFalse(self.result()["passed"])
    def test_source_changed_after_inventory(self):
        (self.root/"src/core.jl").write_text("f()=2\n");self.assertFalse(self.result()["passed"])
    def test_truncated_counts(self):
        (self.root/"src/core.jl.1.cov").write_text("        1 x=1\n");self.assertFalse(self.result()["passed"])
    def test_wrong_source_in_cov(self):
        p=self.root/"src/core.jl.1.cov";p.write_text(p.read_text().replace("value_1","bad_1"));self.assertFalse(self.result()["passed"])
    def test_unmeasured_function_cannot_vanish(self):
        p=self.root/"ext/adapter.jl.1.cov";p.write_text(p.read_text().replace("        1 ","        - "))
        self.assertTrue(any("Unmeasured function" in s for s in self.result()["failures"]))
    def test_zero_counts_are_not_execution(self):
        self.add("ext/adapter.jl",10,0);self.assertFalse(self.result()["passed"])
    def test_stale_output_is_refused(self):
        scripts=self.root/"scripts";scripts.mkdir()
        runner=pathlib.Path(__file__).resolve().parents[1]/"scripts/run_coverage.py"
        shutil.copyfile(runner,scripts/runner.name)
        out=self.root/"out";out.mkdir();(out/"report.json").write_text('{"passed":true}')
        result=subprocess.run([sys.executable,str(scripts/runner.name),"--core-project",str(self.root),"--integration-project",str(self.root),"--output",str(out)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        self.assertNotEqual(result.returncode,0)
        self.assertIn(b"Coverage output must be empty",result.stdout)
        self.assertFalse((out/"stages.json").exists())
    def test_failed_stage_never_reaches_gate(self):
        scripts=self.root/"scripts";scripts.mkdir()
        runner=pathlib.Path(__file__).resolve().parents[1]/"scripts/run_coverage.py"
        shutil.copyfile(runner,scripts/runner.name)
        fake=self.root/"fail-julia";fake.write_text("#!/bin/sh\nexit 17\n");fake.chmod(0o700)
        out=self.root/"out"
        result=subprocess.run([sys.executable,str(scripts/runner.name),"--core-project",str(self.root),"--integration-project",str(self.root),"--output",str(out)],env={**os.environ,"JULIA":str(fake)},stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        self.assertEqual(result.returncode,17)
        self.assertEqual(json.loads((out/"stages.json").read_text()),[dict(stage="inventory",exit_code=17)])
        self.assertFalse((out/"report.json").exists())
if __name__=="__main__":unittest.main()
