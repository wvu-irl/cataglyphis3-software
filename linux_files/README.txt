In order to reserve cores 0 and 1 for the hardware interface and the navigation filter the following must be add to GRUB_CMDLINE_LINUX_DEFAULT

isolcpus=0,1

Then execute

sudo update-grub

Note that each thread is considered a core for isolcpus and taskset, so the i7 has 8 cores labeled 0,1,2,3,4,5,6,7.
