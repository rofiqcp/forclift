from pathlib import Path
import yaml
from navigation_runtime.runtime_schema import (
    _merge_missing, activate_staged_profile, restore_profile_backup,
)


def dump(path, data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')


def load(path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def test_merge_missing_preserves_tuning():
    active={'controller':{'vx_max':0.63}}
    default={'controller':{'vx_max':1.0,'batch_size':400},'new_block':{'enabled':True}}
    changed,added=_merge_missing(active,default)
    assert changed
    assert active['controller']['vx_max']==0.63
    assert active['controller']['batch_size']==400
    assert active['new_block']['enabled'] is True
    assert 'controller.batch_size' in added


def test_profile_post_validation_rollback_removes_new_files(tmp_path: Path):
    active=tmp_path/'runtime'/'navigation'; staged=tmp_path/'stage'/'navigation'; backup=tmp_path/'backup'
    dump(active/'nav.yaml',{'v':1,'keep':'tuning'})
    dump(staged/'nav.yaml',{'v':2})
    dump(staged/'new.yaml',{'x':3})
    pre=activate_staged_profile({'navigation':staged},{'navigation':active},backup)
    assert load(active/'nav.yaml')['v']==2
    assert (active/'new.yaml').exists()
    # Simulate a semantic post-validation failure.
    restore_profile_backup({'navigation':active},backup,pre)
    assert load(active/'nav.yaml')=={'v':1,'keep':'tuning'}
    assert not (active/'new.yaml').exists()
