"""Verify published release bytes and record an SSH-readable immutable receipt."""
import hashlib,json,os,subprocess,tempfile
from pathlib import Path

def run(*args):
    return subprocess.run(args,check=True,text=True,capture_output=True).stdout.strip()

tag=os.environ['RELEASE_TAG']
package=os.environ['RELEASE_PACKAGE']
assets=sorted(p for p in Path('dist').iterdir() if p.suffix in ('.whl','.gz'))
assert len(assets)==2
hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in assets}
with tempfile.TemporaryDirectory() as tmp:
    run('gh','release','download',tag,'--dir',tmp)
    for name,digest in hashes.items():
        assert hashlib.sha256((Path(tmp)/name).read_bytes()).hexdigest()==digest
remote=json.loads(run('gh','release','view',tag,'--json','url,tagName'))
receipt={'tag':tag,'source_sha':run('git','rev-parse','HEAD'),'release_url':remote['url'],'assets_sha256':hashes}
Path('release-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
ref='refs/notes/releases/'+package
existing=subprocess.run(['git','ls-remote','--exit-code','origin',ref],capture_output=True).returncode==0
if existing:run('git','fetch','origin',ref+':'+ref)
run('git','config','user.name','github-actions[bot]')
run('git','config','user.email','41898282+github-actions[bot]@users.noreply.github.com')
run('git','notes','--ref='+ref,'add','-F','release-receipt.json',receipt['source_sha'])
run('git','push','origin',ref+':'+ref)
