import json
from space_arm_platform.dataset_view import Store


def test_listing_keeps_legacy_and_reports_private_batch(tmp_path):
    for name,meta in [('old',{}),('new',{'scene_instance':{'collection_batch':{'batch_id':'2026-a','group_id':'two'}}})]:
        p=tmp_path/name;p.mkdir();(p/'metadata.json').write_text(json.dumps(meta))
    rows={r['id']:r for r in Store(tmp_path).listing()['episodes']}
    assert rows['old']['collection_batch']=={}
    assert rows['new']['collection_batch']=={'batch_id':'2026-a','group_id':'two'}
