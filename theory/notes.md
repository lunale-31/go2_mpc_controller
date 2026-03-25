# Notes

## Motion S

### init() (?)
```
[INFO] [1774456500.291338812] [monitor]: MS Request:
header:
  identity:
    id: 54016616987116
    api_id: 2
  lease:
    id: 0
  policy:
    priority: 0
    noreply: false
parameter: ""
binary: []

[INFO] [1774456500.293112857] [monitor]: MS Response:
header:
  identity:
    id: 54016616987116
    api_id: 2
  status:
    code: 3203
data: ""
binary: []
```

### get_silent()
```
[INFO] [1774456500.294932727] [monitor]: MS Request:
header:
  identity:
    id: 54016619927048
    api_id: 1005
  lease:
    id: 0
  policy:
    priority: 0
    noreply: false
parameter: ""
binary: []

[INFO] [1774456500.295209281] [monitor]: MS Response:
header:
  identity:
    id: 54016619927048
    api_id: 1005
  status:
    code: 0
data: "{\"silent\":false}"
binary: []
```

### set_silent(bool)
```
[INFO] [1774456500.296683379] [monitor]: MS Request:
header:
  identity:
    id: 54016622039194
    api_id: 1004
  lease:
    id: 0
  policy:
    priority: 0
    noreply: false
parameter: "{\"silent\":true}"
binary: []

[INFO] [1774456500.299814814] [monitor]: MS Response:
header:
  identity:
    id: 54016622039194
    api_id: 1004
  status:
    code: 0
data: ""
binary: []

[INFO] [1774456500.300979301] [monitor]: MS Request:
header:
  identity:
    id: 54016626714320
    api_id: 1004
  lease:
    id: 0
  policy:
    priority: 0
    noreply: false
parameter: "{\"silent\":false}"
binary: []

[INFO] [1774456500.302856133] [monitor]: MS Response:
header:
  identity:
    id: 54016626714320
    api_id: 1004
  status:
    code: 0
data: ""
binary: []
```