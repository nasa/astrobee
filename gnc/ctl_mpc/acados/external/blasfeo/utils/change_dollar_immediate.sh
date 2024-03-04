#! /bin/bash

#for file in `find kernel/ -name "*.S"`; do echo $file; done

for file in `find kernel/ -name "*.S"`; do sed -i 's/\$0/\$ 0/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$1/\$ 1/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$2/\$ 2/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$3/\$ 3/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$4/\$ 4/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$5/\$ 5/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$6/\$ 6/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$7/\$ 7/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$8/\$ 8/g' $file; done
for file in `find kernel/ -name "*.S"`; do sed -i 's/\$9/\$ 9/g' $file; done
