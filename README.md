# skia-sentinel-firmware

Various drivers for the Skia Sentinel, a tentative Air Quality Monitor "framework."

## Development setup

You need a Linux machine, or Git Bash, or a similar environment (with a POSIX shell, coreutils, etc).

### Initializing DevContainer environment

This only needs to be executed once.

Execute:

```sh
./scripts/devcontainer-init.sh
```

#### Updating DevContainer environment with changes from upstream

Anytime a new devcontainer configuration is changed and you need to regenerate devcontainer.json and any other supporting file. Just beware that any changes done to devcontainer-related files **will be overwritten**. This includes the following files:

- [`devcontainer.json`](/.devcontainer/devcontainer.json)
- [`devcontainer.env`](/.devcontainer/devcontainer.env)
- [`docker-compose.yml`](/.devcontainer/docker-compose.yml)

None of those files are tracked by Git. If you made any changes to them in order to adapt to your local environment, it's advisable to either make a backup before updating, or introduce the changes manually.

Regardless, in order to re-generate files, just execute:

```sh
./scripts/devcontainer-init.sh -p

# Optionally:
./scripts/devcontainer-init.sh --pristine
```


### Initializing development environment


In case you're running the project inside a devcontainer, initialize the repository first. See the previous section for details on such a procedure.

Initializing the project only needs to be executed once.

Execute:

> **NOTE:** This command should be performed **INSIDE** your development environment (i.e. the devcontainer, if you're using it).

```sh
./scripts/dev-init.sh
```

In case the script is changed (e.g. to add support to some toolchain, or to install blobs needed by the boards the project supports, etc.), you can just execute it again. All it does is initialize/update West and Zephyr, among other trivial things.


### Building the project

Execute:

```sh
./scripts/dev-build.sh
```

### Flashing to hardware

Only the Wemos Lolin C3 Mini is supported, at the moment. But you can change the board by setting the `BOARD` variable in the [`.env` file](.env) (this file is created by `dev-init.sh`, based on [`.env.sample` file](.env.sample)).

Execute:

```sh
./scripts/dev-flash.sh
```
