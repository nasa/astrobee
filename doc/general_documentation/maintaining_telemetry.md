
# Maintaining Astrobee Telemetry Backward Compatibility

## Motivation and scope

The Astrobee Facility has built up a valuable archive of telemetry bags
recorded by Astrobee robots on the ISS since commissioning in April
2019. Astrobee's flight software (FSW) has also changed substantially over
that time, including several changes to the definitions of the message
types contained in the telemetry bags. As a result, it has become more
difficult over time to analyze or play back older legacy bags with the
latest version of FSW.

This document establishes objectives for backward compatibility and
proposes a maintenance approach.

## Telemetry maintenance objectives

1. Astrobee developers should be able to `rosbag play` archived Astrobee
   ISS telemetry bags with the latest FSW, as a way to test the software
   in the ISS environment. For example, localization system development
   has relied heavily on bag playback as a way to iterate and overcome
   challenges related to the complex ISS interior that can't be
   replicated on the ground.

2. External users without prior ROS experience should be able to analyze
   archived Astrobee ISS telemetry bags with minimal training by
   following simple, documented processes. These users could be Astrobee
   guest scientists or could be third parties with less Astrobee
   background and insider access who are opportunistically finding new
   uses for our archived data. (For example, making new observations in
   our ISS imagery.)

3. We also have a priority list of bad outcomes to avoid:

    - Misleading users. If the message semantics has changed such that
      trying to interpret legacy messages with the new semantics is
      actively misleading, this is the worst case.

    - Breaking all messages. Some bag analysis tools are sensitive and
      crash if the bag has even a single problematic message definition.
      We should strive hard to avoid this problem.

    - Loss of ability to analyze specific message types or specific
      fields. This may be more acceptable, depending on the specific
      user needs.

4. We should minimize the burden on Astrobee developers and the Astrobee
   Facility ops team required to realize these goals.

## Telemetry maintenance requirements

1. Developers should have guidance about steps they can take to maintain
   legacy bag compatibility with minimum effort (this document).

2. Any required bag processing should be fully automated to the greatest
   extent possible, minimizing burden on the Astrobee Facility and
   users.

3. Bags processed for developers or external users to work with should
   pass `rosbag check`.  This tool applies a number of tests, but is
   principally designed to check that the bag's messages can be played
   back with the latest version of the FSW ROS environment (the message
   definitions are compatible for all of the message types recorded in
   the bag).

4. Along the same lines, another test bags should pass is `rostopic echo
   -b <bag> -a`. This forces message deserialization.

4. Where message semantics have changed, we should either remap the
   legacy message contents to accurately reflect the latest semantics,
   or rename the legacy message type to clarify that it can't be
   interpreted like the current type.

5. ROS message types defined by guest scientists:

    - If legacy bag compatibility is required for these messages, it
      should be the responsibility of the guest science developers.

    - As a fallback, the FSW team should provide a tool for filtering
      out no-longer-compatible guest science messages, in case the other
      messages in the bag are still valuable.

6. Out of scope: The primary focus is on legacy messages recorded in ISS
   telemetry bags, or particularly important ground experiments.  We
   will avoid investing effort in backward compatibility with old
   message definitions that were never recorded on the ISS. For example,
   certain older versions of Astrobee perching messages were tested on
   the ground, but never deployed on the ISS, and in those cases
   backward compatibility is not important.

## Telemetry maintenance approach

We will provide a fully automated script that can fix arbitrary legacy
ISS Astrobee bags to be compatible with the latest FSW.

The starting point for the script is the standard [`rosbag`
migration](http://wiki.ros.org/rosbag/migration) system provided with
ROS. If we invest in maintaining the necessary `*.bmr` bag migration
rules, the `rosbag fix` script can fix most but not all message
definition problems.

One remaining problem is incomplete message definitions in messages
published by `rosjava` nodes. We have a script that can fix these
message definitions. It must be run before `rosbag fix` in order to
avoid a crash.

The other remaining problem arises when there are substantial changes in
a message definition that make migration to the new definition
problematic. For example:

- If a field has been deleted, any attempt to migrate an old message
  instance to the new definition will necessarily involve data loss.

- If the change is sufficiently complicated, it may not be worth the
  effort to figure out how to reliably remap old data into the new
  structure.

When migration is problematic, we will instead "freeze" the old message
definition by copying it to a legacy message type name, and rename the
message type in legacy bags to use the frozen type name.

## Developer guidelines for modifying messages

- Begin by trying hard to get the initial design of any new messages
  correct on the first pass, avoiding later changes that will cause a
  migration hassle.

- However, you can change a message's definition without worrying about
  backward compatibility considerations up until the point when precious
  (ISS) data is recorded.

- Try to avoid message definition changes that are problematic for
  backward compatibility.

   - Adding a field is easy. However, if there is legacy data that needs
     to be migrated, try to design the new field to accommodate a clear
     no-data value that can be used to migrate old messages that didn't
     contain the field.  (For example, rather than a `bool` field, you
     might prefer to use an `int8` field with enumerated values that
     include an explicit no-data value.)

   - Deleting a field is problematic for migration, assuming the field
     had any value when recorded in legacy messages. (Migrating to the
     new message type could lose precious data.) This might or might not
     be acceptable, depending on the situation.

   - Try hard to avoid renumbering the label values for enumerated
     types. If you do that, message migration will have to remap the
     legacy values to the new values (which is complicated and
     error-prone), or risk keeping values that are actively misleading
     for data analysis. Instead, you should assign never-before-used
     values as needed for new labels. If some old labels are no longer
     used in new messages, you should still keep them in the message
     definition as documentation for interpreting legacy
     messages. However, it's encouraged to clarify their legacy status
     (e.g., add a comment, or even prepend `OLD_` to the label text).

   - If the message changes are so involved that you can't figure out
     how you would migrate old message data to the new message type,
     consider whether it might be better to simply define a brand-new
     message type.

- Legacy message types that were recorded in precious (ISS) bags but are
  no longer published by the current software should not be deleted by
  default, because they remain useful for backward compatibility with
  the archived bags. Instead, document the legacy status of the message
  (e.g., add it to a list of messages that are only present for backward
  compatibility).

- When you make a message definition change that is compatible with
  `rosbag` migration, please define a migration rule for the message at
  the same time, following the [migration
  instructions](http://wiki.ros.org/rosbag/migration), ideally using
  section 3.1 "Preferred Approach".

   - Place your `*.bmr` file in the `bmr` folder of the appropriate ROS
     package. See
     [`ff_msgs/bmr`](https://github.com/nasa/astrobee/tree/develop/communications/ff_msgs/bmr)
     for an example of naming and styling conventions.

   - It is not necessary to export your `*.bmr` file in `package.xml` as
     documented in section 4. Unfortunately, that helps only if `rosdep`
     is configured on your host, which we can't always count on.
     Instead, our automated bag processing keeps a list of folders to
     search for migration rules.

   - Once your `*.bmr` file is added, `rosbag_fix_all.py` will apply
     your migration rules as needed.

   - Verify your new rule, as described below.

- If you are contemplating a new message definition change that is too
  involved for migration to be feasible, you should probably just define
  a brand-new message type instead and avoid any migration
  hassle. However, we already have past message changes of this flavor
  built up as technical debt. We deal with them by "freezing" a legacy
  copy of the old message definition. Follow these conventions:

   - Messages are frozen by copying their definition files (e.g.,
     `*.msg` or `*.action`).

   - To avoid clutter, the frozen copies may be placed in their own
     package, for example `ff_legacy_msgs` for messages from
     `ff_msgs` or `ff_hw_msgs`.

   - Files defining frozen messages should be named by appending a
     version number to the original filename. For example,
     `Inspection.action` would become `InspectionV1.action`.

   - When you freeze a parent type, you should also freeze any subtypes
     contained in its fields, if there is any chance their definition
     could change in the future.

   - Add a rule for renaming the message type, as follows:

        - Append your rule to the list in the `rename_types` field of
          the file named `bmr/rewrite_types_rules.json` within the
          relevant ROS package, such as `ff_msgs`.

        - Your rule must specify these four fields:

            - `old_type`: The old message type name in
              `some_pkg/SomeMsg` format.

            - `old_type_md5sum`: You can determine the MD5 sum for the
              old type by examining the output of `rosbag info` run on a
              bag that contains the legacy message, or by running
              `rosmsg md5 <old_type>` with the correct legacy version of
              FSW checked out.

            - `new_type`: The new (frozen) message type name.

            - `new_type_md5sum`: Determine with `rosmsg md5 <new_type>`.

        - The rule is interpreted as follows: if a bag contains messages
          that match both `type = old_type` and `type_md5sum =
          old_type_md5sum`, those messages will have their type and MD5
          sum information rewritten to the new values.

    - If you created a new rules file for the package, you may need to
      add it to the list of rules files that `Makefile.rosbag_fix_all`
      looks for.

    - Once your renaming rule is added, `rosbag_fix_all.py` will apply
      it as needed.

    - TODO: Point to an example. (There is already an example written,
      but it will be merged separately later.)

    - Verify your new rule as described below.

- Verifying message rewrite rules:

    - The same approach applies, whether you wrote a migration rule or a
      type renaming rule.

    - Run `rosbag_fix_all.py` on an archived bag containing the legacy
      message.  It automatically applies various consistency checks.

    - Run `rostopic echo -b <bag> -p <topic>` on the fixed bag and spot
      check the field values in the output CSV file.

## Ops approach for fixing bags

As long as FSW keeps evolving with changing message definitions, no
matter how many times we apply our automated fix script to an archived
bag, a future FSW change could always break its compatibility again.

Therefore, we should take a layered approach to fixing bags:

- Whenever we make some use of an archived bag, particularly when we
  publicly release the data, we should try to fix the bag at that time
  (and create any retroactive migration rules as needed to support
  that). Regular usage of the fixing script should also help to avoid
  bit-rot.

- Since future FSW changes will continue to break bags, and we don't
  want to take on the burden of reprocessing all the previously released
  bags after every breaking message definition change, we should also
  provide a simple, documented approach for external users to fix them.

Usage instructions for fixing bags are documented in: \ref
using_telemetry

## Complementary backward compatibility approach

This document focuses on maintaining backward compatibility of legacy
bags with the ROS tools such as `rosbag` and `rostopic` that need the
old messages to be compatible with the latest message
definition. Clearly, this simplifies uses like testing the latest FSW
using legacy bag playback.

However, if all you need is offline analysis outside the ROS tools, the
[`bagpy` library](https://jmscslgroup.github.io/bagpy/) for Python 3 is
a viable alternative. It relies on the fact that the `rosbag` format
includes complete embedded message definitions, allowing it to
deserialize the binary messages and output CSV with named fields, even
if the latest ROS packages that define the messages no longer have
consistent message definitions. In fact, it doesn't even require a full
ROS installation.  Fixed bags will never break from the `bagpy` analysis
perspective.  Using `bagpy` may be especially attractive for users
who lack prior ROS experience.

## Retroactive migration technical debt

Since Astrobee went through ISS commissioning in April 2019, there have
been several message definition changes that lack corresponding
migration rules.

Our developers should retroactively create the necessary migration rules
on a lazy as-needed basis. The most common triggering event to create a
new rule would be noticing that `rosbag_fix_all.py` fails on an archived
bag during the public data release process.
