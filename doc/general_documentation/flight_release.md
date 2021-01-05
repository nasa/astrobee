\page release Creating a Flight Release

# Creating a Flight Release

This page describes the steps required to complete a flight release. It assumes
that your environment is configured to cross-compile the fsw for the `armhf`
architecture.

## Create a Release Branch

Release branches will be created on a user's fork. To make a release branch,
checkout and update develop. Then make a release branch off of develop.

    git checkout develop
    git fetch upstream
    git merge upstream/develop
    git checkout -b release-0.x.x


## Update the Release Version

The release version must be updated before creating the Debian. Please use
the update release script to do this. Before running the script, get a list of
features that have been added since the last release. This list will need to be
added to the change log that gets edited when the script is run. Also, in the
change log, change UNRELEASED to testing.

    ./scripts/setup/debians/update_release.md 0.x.x


*Note: Make sure that your EDITOR variable is set to your preferred editor
before invoking this script. Using vim seems to generate an unimportant error.*

## Commit and Push Version Change

Please make sure to commit and push all the files that were changed.

    git commit -m "meaninful message" files_that_changed
    git push -u origin release-0.x.x


## Create the Debian package


    ./scripts/build/build_debian.sh


## Testing the Release

On confluence, there is a [release testing procedure](https://babelfish.arc.nasa.gov/confluence/display/FFFSW/IRG-FFTEST302+-+Astrobee+Release+Testing).
This procedure will walk you through all the testing that needs to be done to
make sure the debian is flight ready. Please create a copy of the procedure by
clicking on the ellipsis at the top right corner of the page and selecting copy.
In the popup, in the 'Parent page' text box, please enter 'Executed Procedure'
and click 'Copy'. Remove 'Copy of' from the title and add the version and date
after the test number such that it looks like
'IRG-FFTEST302 - 0.x.x -YYYY-MM-DD - Astrobee Release Testing'. Be sure to fill
out the 'CAs-Ran Summary' table and the 'Result' and 'Notes' column for every
step. After running through the procedure, please provide a brief summary of
your results in the
[release testing results table](https://babelfish.arc.nasa.gov/confluence/display/FFFSW/Release+Testing+Results).

## Fix Bugs

If there are bugs that arise during testing, please fix the bugs, push the
fixes, recreate the debian, and repeat the release testing procedure.

## Finish Release

Once the debian passes all the required testing, it is ready to become an
official flight release. Please make a pull request on astrobee develop and set
Brian, Katie, or Marina as the reviewer. They will review the request and merge
it into develop and master. You will also need to copy the debian to a temporary
location on volar.

    scp astrobee0_0.x.x_armhf.deb \
      <ndc_username>@volar:/home/p-free-flyer/free-flyer/FSW/ars_debs/dists/xenial/main/release_candidate/

After the debian has been copied to volar, make sure the group permissions are
set to read and right. Finally, please email Ruben Garcia Ruiz so that he can
sign it and stage it.

### Merging into Develop and Master

This section is for reviewers only! After reviewing the pull request and
verifying that the release passed all sections of the release testing procedure,
please merge the pull request into develop. You will then need to merge the
release branch into master as well. After this merge, on master, please create
an annotated tag with a descriptive message of the most noteable new features.
