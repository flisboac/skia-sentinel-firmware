# skia-sentinel-firmware

Various drivers for the Skia Sentinel, a tentative Air Quality Monitor "framework."

## Development setup

You need a Linux machine, or Git Bash, or a similar environment (with a POSIX shell, coreutils, etc).

### Initializing development environment

This only needs to be executed once.

Execute:

```sh
./dev-init.sh
```

### Building the project

Execute:

```sh
./dev-build.sh
```

### Flashing to hardware

Only the Wemos Lolin C3 Mini is supported, at the moment. But you can change the board by setting the `BOARD` variable in the [`.env` file](.env) (this file is created by `dev-init.sh`, based on [`.env.sample` file](.env.sample)).

Execute:

```sh
./dev-flash.sh
```
