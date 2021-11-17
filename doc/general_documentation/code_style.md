\page astrobee-code-style Astrobee code style

The Astrobee code uses [cpplint](https://en.wikipedia.org/wiki/Cpplint) to enforce a consistent and uniform coding style. Code which fails this tool cannot be committed with Git. It is suggested that the [clang-format-8](https://launchpad.net/ubuntu/bionic/+package/clang-format-8) program be installed on your machine. The *git commit* command will invoke this tool which will fix most (but not all) of the formatting errors using as a guide the ``.clang-format`` style file at the base of the Astrobee repository.
