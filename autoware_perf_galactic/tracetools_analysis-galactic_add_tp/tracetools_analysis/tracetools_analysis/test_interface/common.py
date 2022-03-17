from enum import Enum

class Result(Enum):
    DESIRED = 1,
    ACCEPTABLE = 2,
    NOT_ACCEPTABLE = 3
    def __str__(self):
        return self.name.lower()

class Unit():
    def __init__(self, name):
        assert name in ['ms', 'us', 'ns']
        self._name = name
        self._scale = 1
        if name == 'ms':
            self._scale = 1.0e-3
        elif name == 'us':
            self._scale = 1.0e-6
        elif name == 'ns':
            self._scale =1.0e-9
        
    @property
    def scale(self):
        return self._scale
    
    def __str__(self):
        return self._name

def prepare_dir(path):
    import os
    dir_path = os.path.dirname(path)
    print(dir_path)
    if dir_path in ['', '.']:
        return

    os.makedirs(os.path.dirname(dir_path), exist_ok=True)

def read_yaml(yaml_path):
    import yaml
    with open(yaml_path, 'r') as f:
        obj = yaml.safe_load(f)
    return obj

def write_yaml(yaml_path, obj):
    import yaml
    import os

    prepare_dir(yaml_path)
    with open(yaml_path, 'w') as f:
        yaml.dump(obj, f)
