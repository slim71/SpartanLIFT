# Naming convention

## Actions taken by a secondary module

`gather[...]`
A secondary module asks data owned by another module.
This will go through a pass-through method of the main module if the target data is owned by another seconday module.

`confirm[...]`
A secondary module checks a flag owned by the main module.

`signal[...]`
A secondary module triggers a change of data in another module.
This will go through a pass-through method of the main module if the target data is owned by another seconday module.
**Note**: the `signalHandler` function is excluded from this behavior.

## Actions taken by the Main module

`commence[...]`
The main module actually initiates operations triggered by a secondary module.

`is[...]`
The main module computes and returns a flag to the requiring module.

`request[...]`
The main module asks data to a secondary module.

## Common methods

`get[...]`
The module owning the requested data returns it to the requiring module or to the pass-through method of the main module.

## Response patterns

| Initiator |     Receiver    |
|-----------|-----------------|
| `signal`  |     `commence`  |
| `request` |      `get`      |
| `confirm` |      `is`       |
| `gather`  | `request`/`get` |
