# nardeas/firmware/python

**Note:** This repository is NOT meant to be installed as module. Instead you can use git submodules to include the reference implementations in your design:

> Example structure for `<username>/<repository>`:
```
ext/
src/
  |-- lib/
    |-- devices/
.gitmodules
.gitignore
README.md
```

**Step 1:** Include submodule

```
pushd ./ext
git submodule add git@github.com:nardeas/firmware.git
popd
```

**Step 2:** Create submodule symlinks

```
pushd src/lib/devices
ln -s ../../../ext/firmware/python/micropython/devices/<filename>.py
popd
```

**Step 3:** Commit submodule links 

```
git add .gitmodules src/lib/devices/<filename>.py
git commit -m "Add submodule symlinks" 
```
