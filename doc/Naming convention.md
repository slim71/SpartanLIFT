# Naming convention

As a structure, I've decided no direct communications among modules is to be done. Everything passes though the main module and is redirected to the appropriate one

## Actions taken by a secondary module

`gather[...]`
A secondary module asks data owned by another module.
This will go through a pass-through method of the main module if the target data is owned by another secondary module.

`confirm[...]`
A secondary module checks a flag owned by the main module.

`signal[...]`
A secondary module triggers a change of data in another module.
This will go through a pass-through method of the main module if the target data is owned by another seconday module.
**Note**: the `signalHandler` function is excluded from this behavior.

## Actions taken by the Main module

`commence[...]`
The main module actually initiates operations triggered by another module.

`initiate[...]`
The main module itself activates a functionality carried on by another module.

`request[...]`
The main module asks data to a secondary module.

## Common methods

`get[...]`
The module owning the requested data returns it to the requiring module or to the pass-through method of the main module.

`is[...]`
The module owning a flag returns said flag to the requiring module (another one or itself).

## Response patterns

| Source (Initiator)        | Passthrough (main) | Destination            |
| ------------------------- | ------------------ | ---------------------- |
| `signal[action]` (ext)    | `commence[action]` | '[action]' (ext/main)  |
| `gather[data]`  (ext)     | `request[data]`    | `get[data]` (ext/main) |
| `confirm[flag]` (ext)     | N/A                | `is[flag]` (any)       |
| `request[data]` (main)    | N/A                | `get[data]` (ext)      |
| `initiate[action]` (main) | N/A                | '[action]' (ext)       |

ext: external module
main: main module (PelicanModule)
