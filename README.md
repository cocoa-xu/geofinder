# geofinder

Adding a `country` info to each image in the iNaturist annotation JSON file.

### Usage

```shell
$ geofinder -b borders.json -a train.json -o train-country.json --verbose

[+] Loading border JSON file from borders.json...
[+] Building R-Tree...
[+] Loading annotation JSON file from train.json...
[+] Querying for 2686843 images...
[-] Current querying for image 2686842...
[+] Writing queried results to train-country.json...
[+] Processing time: 358.692860s
[+] Total=2686843, valid=2678609, failed=798101
```
