\page release Generate GitHub release

# Generate releases and synching with Github

## Create a release branch

1. Test that all new features are functional in all environments

2. Create a branch release
```
git checkout develop
git pull
git checkout -b release-0.x.0
```

3. Update the version number and release notes: ./scripts/setup/debians/update\_release.sh VERSION\_NUMBER
```
git commit -m "Bumped version number to 0.1.x"
```

4. Push the branch to the server
```
git push -u origin release-0.x.0
```

5. Fold the release into master and tag
```
git checkout master
git merge --no-ff release-0.1.x
# Add message: applied release-0.1.x to master
git push origin master
git tag -a v0.1.x
# Add message: tagged new version... more descriptive message"
git push origin tag v0.1.x
```

6. Integrate the release into develop
```
git checkout develop
git merge --no-ff release-0.1.x
# Add a message: applied release-0.1.x to develop
git push origin develop
```

## Propagate a release/hotfix to Github

### Add remote (only if first time)
```
git remote add github git@github.com:nasa/astrobee.git
```
(keys need to be setup correctly)

### Sync the repos

1. Work on the public branch
```
git checkout public
```

2. Squash history between releases
```
git merge --squash master
git commit
# Remove all the summary commit
# Add a meaningful message for the public repo with summary
# and detailed message
```

3. Push to github
```
git push github public:master
```

4. Push to babelfish
```
git push origin public
```

5. Sync github with babelfish
```
git checkout master
git merge --no-ff public
# Just synchronization, no message necessary
git push origin master
git checkout develop
git merge --no-ff public
# Just synchronization, no message necessary
git push origin develop
```
