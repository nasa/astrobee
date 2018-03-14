# Generate releases and synching with Github

## Generate Hotfix with GitFlow

1. Create hotfix branch from master
```
git checkout master
git pull
git checkout -b hotfix-0.1.x
```

1. Apply the fix
  1. Make changes
```
git commit -m"meaningful message"
```
  1. Update the version number and release notes: ./scripts/setup/debians/update\_release.sh VERSION\_NUMBER
```
git commit -m"bumped version number to 0.1.x"
```

1. Fold the hotfix into master and tag
```
git checkout master
git merge --no-ff hotfix-0.1.x
# Add message: applied hotfix 0.1.x to master
git push origin master
git tag -a v0.1.x
# Add message: tagged new version... more descriptive message"
git push origin tag v0.1.x
```

1. Integrate the hotfix into develop
```
git checkout develop
git merge --no-ff hotfix-0.1.x
# Add a message: applied hotfix 0.1.x to develop
git push origin develop
```

## Propagate a release/hotfix to Github

### Add remote (only if first time)
```
git remove add github git@github.co:nasa/astrobee.git
```
(keys need to be setup correctly)

### Sync the repos

1. Work on the public branch
```
git checkout public
```

1. Squash history between releases
```
git merge --squash master
git commit
# Remove all the summary commit
# Add a meaningful message for the public repo with summary
# and detailed message
```

1. Push to github
```
git push github public:master
```

1. Push to babelfish
```
git push origin public
```

1. Sync github with babelfish
```
git checkout master
git merge --no-ff public
# Just synchronization, no message necessary
git push origin master
```
