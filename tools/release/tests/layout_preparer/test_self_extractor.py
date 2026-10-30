from pathlib import Path
import hashlib,io,subprocess,tarfile,tempfile
root=Path(__file__).resolve().parents[2]/'layout_preparer'
template=(root/'self-extractor.template').read_text()
with tempfile.TemporaryDirectory(prefix='spacesim-extractor-test-') as temp:
    base=Path(temp)
    stream=io.BytesIO()
    with tarfile.open(fileobj=stream,mode='w:gz') as archive:
        body=b'#!/bin/bash\nprintf "LAUNCH:%s\\n" "$*"\n'
        info=tarfile.TarInfo('space-sim-layout-preparer/space-sim-layouts')
        info.mode=0o755;info.size=len(body)
        archive.addfile(info,io.BytesIO(body))
    payload=stream.getvalue()
    sha=hashlib.sha256(payload).hexdigest()
    header=template.replace('@SHA@',sha).replace('@ID@','test-'+sha[:12])
    header=header.replace('@LINE@',str(header.count('\n')+1))
    script=base/'test.run';script.write_bytes(header.encode()+payload);script.chmod(0o755)
    def run(*args):
        return subprocess.run([str(script),*map(str,args)],text=True,capture_output=True)
    dest=base/'directory with spaces'
    result=run('--install-dir',dest,'check')
    assert result.returncode==0,(result.stdout,result.stderr)
    assert 'LAUNCH:check' in result.stdout
    result=run('--install-dir',dest,'status','--output','job with spaces')
    assert result.returncode==0 and 'Using ' in result.stdout and 'LAUNCH:status' in result.stdout
    unrelated=base/'unrelated';unrelated.mkdir();(unrelated/'keep').write_text('keep')
    result=run('--extract',unrelated)
    assert result.returncode!=0 and (unrelated/'keep').read_text()=='keep'
    second=base/'extract only'
    result=run('--extract',second)
    assert result.returncode==0 and 'LAUNCH:' not in result.stdout
    script.write_bytes(header.encode()+payload+b'corrupt')
    result=run('--extract',base/'corrupt')
    assert result.returncode!=0 and 'checksum mismatch' in result.stderr
    assert not (base/'corrupt').exists()
print('PASS: extraction, spaces, reuse, extract-only, existing-directory protection, corruption rejection')
