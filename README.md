# geofinder

Adding a `country` info to each image in the iNaturist annotation JSON file.

This software is developed for my own usage when pre-processing the iNaturist daya, and if it happens to be useful to you, I'll be very glad and hope it can help you.

### Usage

```shell
$ geofinder -b borders.json -a train.json -o train-country.json --verbose

[+] Loading border JSON file from borders.json...
[+] Building R-Tree...
[+] Loading annotation JSON file from train.json...
[+] Querying for 2686843 images...
[-] Current querying for image 2686842...
[+] Writing queried results to train-country.json...
[+] Processing time: 19.047447s
[+] Total=2686843, valid=2678609
```

Example `train.json` file:
```json
{
    "images": [
        {
            "longitude": -4.25,
            "latitude": 55.86
        }
    ]
}
```

Corresponding output JSON:
```json
{"images":[{"country":"GB","latitude":55.86,"longitude":-4.25}]}
```
