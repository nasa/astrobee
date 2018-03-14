** Setup **

In this directory, add the following files:

- `aptly`: Download the aptly software from https://www.aptly.info
- `astrobee_conf`: Directory containing the latest multistrap config files from
  `submodules/platform/rootfs`. This can be a symlink.
- `aptly.conf`: config file for aptly. On volar, we override the rootDir.

Update the Astrobee Debian files in:

`$ASTROBEE_DEBIAN_DIR/binary-armhf`
`$ASTROBEE_DEBIAN_DIR/binary-amd64`

By default, `$ASTROBEE_DEBIAN_DIR` is
`/home/p-free-flyer/free-flyer/FSW/ars_debs/dists/xenial/main`
for volar.

Also, you must add the Astrobee private key to GPG. When you figure out how to
do this, please update the instructions...

** Creating Initial Repository **

2. Copy the latest `submodules/platform/rootfs/*.conf` files into scripts/astrobee\_conf.
3. Copy the latest debians we made to /home/p-free-flyer/free-flyer/FSW/ars\_debs/dists/xenial/main/binary-armhf and binary-amd64.
4. Run `create_mirrors.sh`. This creates mirrors of the latest ubuntu repository based on the config files.
5. Run `update_all.sh SNAPSHOT_NAME`, choosing an appropriate name. This downloads packages from the mirrors and publishes a snapshot both to this directory and to astrobee.ndc.nasa.gov.

** Updating Only Local Debians **

Install new debians to our local repository. Then run step update\_all.sh.

