\page release Generate GitHub release

# Generate releases and synching with Github

1. Update the version number and release notes: ./scripts/setup/debians/update\_release.sh VERSION\_NUMBER
```
git commit
```

2. Merge into github develop.
3. Merge into github master.
4. Create github release on master with version number.

