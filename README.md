# RFNoC DIFI

rfnoc-difi is an OOT RFNoC block project that allows the x410 to output DIFI packets without any bridge/interrupting software.

- `fpga_image` is a folder containing pre-built fpga bitstreams for the x410 (also useful to trasnfer bitstreams from the lab to the rack).

- `oot_rfnoc_difi` is an out-of-tree RFNoC block that switches the stream from CHDR to DIFI format.

This project targets stock UHD 4.8 (tag `v4.8.0.0`). No special UHD branch is required: the control/data stream separation this project relies on has been part of mainline RFNoC since UHD 4.0.

## Getting Started

It's highly recommended you do development on Linux. It is a much easier setup and faster for development.

There is a ton of Verilog and SystemVerilog within the FPGA design. It is highly recommended to learn Verilog and SystemVerilog basics/syntax if you have limited experience (basics, interface, module, etc.).

### Vivado Setup

It is necessary to download Vivado and get a valid license. For X410 FPGA builds with UHD 4.x, you need Vivado ML Enterprise 2021.1 with the AR76780 patch (this is what UHD's `setupenv.sh` checks for).

[Vivado 2021.1 downloads (archive)](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive.html)

[Vivado AR76780 patch download](https://support.xilinx.com/s/article/76780?language=en_US) (extract the zip into `<Vivado install>/2021.1/patches/AR76780/`; `vivado -version` should then report `2021.1_AR76780`)

If you're on Linux, you can't use more than 3 monitors unless you use the [Vivado AR_72614 patch](https://support.xilinx.com/s/article/72614?language=en_US). 

### UHD Setup

For the build environments, Ettus provides both a [UHD-from-source Guide](https://files.ettus.com/manual/page_build_guide.html) and an [FGPA Guide](https://files.ettus.com/manual/md_usrp3_build_instructions.html).

Check out the UHD release tag matching the UHD version on your hosts (e.g. `git checkout v4.8.0.0`) and use that same tree for the host build, `rfnoc_image_builder`, and the FPGA build, so the FPGA image's compat version matches your UHD driver.

### x410 Setup

The best way to get a grasp of the x410 is via the [x410 User Manual](https://files.ettus.com/manual/page_usrp_x4xx.html) that Ettus provides. 

*Note:* Certain functionalities may be only available on the QSFP ports and not on the Ethernet port. For a stable developmental experience, run any streaming over the QSFP ports and only use the Ethernet port for internet access/ssh connections.

Before continuing, make sure you have (in order):

- Probed the device with `uhd_usrp_probe` (to ensure you can connect)
- SSH'd into the device as root [no password] (to have access to the device)
- Updated the MPM version, file system, motherboard, and SCU on the x410
- Updated the FPGA image

All instructions above should be located within the User Manual.

**If your x410's embedded UHD/MPM version does not match your host UHD version** (normally you should just update the x410 filesystem with `uhd_usrp_update_fs`): you can install a matching MPM onto the x410 from source. Here are the steps to do so:

- (If you have internet connection on the x410) Clone the git repository onto the x410
  - `ssh root@[x410_address]`
  - `git clone https://github.com/whatever/uhddev.git`
- (If you don't have an internet connection on the x410) Enable an ssh connection (below is shown with openssh-server) from the x410 to your host computer and mount the repository over ssh
  - `sudo apt update && sudo apt upgrade && sudo apt install openssh-server`
  - `sudo systemctl status ssh && sudo service ssh status && sudo systemctl enable ssh && sudo systemctl start ssh`
  - `ssh root@[x410_address]`
  - `mkdir uhddev`
  - `sshfs user@yourcomputer:path/to/uhddev uhddev # This will mount ~/path/to/uhddev from the remote machine to ~/uhddev on the x410`
- Install the dev MPM onto the x410
  - `mkdir build_mpm`
  - `cd build_mpm`
  - `cmake -DMPM_DEVICE=x4xx ../uhddev/mpm`
  - `make -j2 install`
- If you used openssh-server you can stop the service on your host machine with `sudo systemctl stop ssh`

If you want to go the route of cross-compiling, you can attempt to adapt [the steps shown here for the N3XX](https://files.ettus.com/manual/page_usrp_n3xx.html#n3xx_software_dev), but this is untested and your mileage may vary.

### Post-setup

After doing this, if you want to build the FPGA image, might as well start it now since it will take a long while. View the [FGPA Guide](https://files.ettus.com/manual/md_usrp3_build_instructions.html) for instructions.

### Building the FPGA image

From `oot_rfnoc_difi/`, with a UHD 4.8 host install (for `rfnoc_image_builder`) and the UHD 4.8 FPGA source tree:

``` bash
rfnoc_image_builder -y icores/x410_100_rfnoc_difi_basic_image_core.yml \
    -F /path/to/uhd/fpga -I . --vivado-path /tools/Xilinx/Vivado
```

For iterative development (e.g. changing only the DIFI block), enable Vivado's
incremental implementation to reuse the previous run's placement/routing:

``` bash
INCR_BUILD=1 rfnoc_image_builder -y icores/x410_100_rfnoc_difi_basic_image_core.yml \
    -F /path/to/uhd/fpga -I . --vivado-path /tools/Xilinx/Vivado
```

This uses the `post_route.dcp` checkpoint from the previous build of the same
image core (kept in `build-<image_core_name>/`), so don't delete that directory
between builds and don't pass `--clean-all`. Two caveats: (1) synthesis still
runs from scratch — only place & route is incremental; (2) after large changes
(new blocks in the image core, big RTL rewrites) or if the post-route timing
report shows violations, delete `build-<image_core_name>/post_route.dcp` and do
a clean build so the reference checkpoint doesn't drift.

## DIFI TX (deframer)

`oot_rfnoc_difi/fpga/rfnoc_block_difi_deframer` implements the TX direction:
the host sends UDP packets of `CHDR header (8B) + DIFI header (28B) + sc16
samples` to a stream endpoint; the deframer strips the DIFI header and
forwards plain CHDR sample packets into a DUC/radio chain (see the
`x410_UC_200_stock64_difi_tx_image_core.yml` image core, which places it on
the RF B:0 TX path).

Any DIFI-legal sample count (>= 1 per packet) is accepted: the deframer
re-packs samples across packet boundaries so that only NIPC-aligned packets
reach the stock DUC (which ignores `tkeep` on its input and would otherwise
append a phantom pad sample on odd packets). A packet with EOB set flushes
the carried residue. Two caveats: packets deliver at least one sample after
the DIFI header, and CHDR timestamps pass through unmodified, so timed
streams are only sample-accurate when packets are already aligned (untimed
streaming recommended for odd counts).

## RFNoC Development

### Introduction

Before starting anything, there are a couple concepts one should understand: what the data looks like in RFNoC, and how the data is transferred betwen places.

The content of the streams being sent around is commonly the CHDR Data packet. You can look at the structure of this in the CHDR_TO_DIFI.md doc and in the RFNoC Specficiation 2.2.1/2.2.2. An important note is that there are two parts to a CHDR Data packet: the header/metadata/timestamp (the context) and the data (the payload).

You might interact with Control packets as well. These are more structured and defined in the RFNoC Specification 2.2.3.

The data is sent between FIFOs, Blocks, Adapters, basically everything with the AXI Protocol. You can learn how [the protocol works here, explained by Xilinx](https://docs.xilinx.com/r/en-US/ug1399-vitis-hls/How-AXI4-Stream-Works).

Within RFNoC blocks, the IO of most everything is AXI-Stream CHDR, and possibly AXI-Stream Payload Context. You can read about these (with very helpful visuals) in Sections 2.3.3.2 and 2.3.3.3 of the RFNoC Specification.

### Understanding RFNoC

Next, it is recommended to go through the [Getting Started with RFNoC in UHD 4.0](https://kb.ettus.com/Getting_Started_with_RFNoC_in_UHD_4.0) tutorial including creating your own RFNoC block. It would be wise to create an RFNoC block that interacts significantly with the AXI-Stream interface (adding metadata is a good trial by fire). Make sure to also create a test bench for the RFNoC block (you can easily copy and adapt Add/Sub or Gain's test bench).

It's really helpful to look at the other blocks that Ettus provided (along with the Gain example) to get a feeling for how some functionalities are implemented (located at `fpga/usrp3/lib/rfnoc/blocks`).

After running through the custom RFNoC block tutorial, run through the `fpga/usrp3/lib` section of the UHD repository and understand the tools that might prove useful. Here are some directories with highlights to get familiar with:

- `axi/ ; axi4_sv/ ; axi4lite_sv/ ; axi4s_sv/` contain common AXI interface declarations and functionalities (e.g. splitting a stream or adding bytes to an AXI-Stream)
- `rfnoc/ ; rfnoc/core/` also contain some RFNoC specific AXI functionalities (such as AXIS CHDR <-> AXIS Payload/Context translators and an stream joiners)
- `rfnoc/xport_sv/ ; rfnoc/xport/` are related to the export of CHDR data over UDP/IPv4 (`xport_sv/chdr_xport_adapter.sv` is especially important in understand the separation of the data/control planes)
- `fpga/usrp3/top/x400/` contains all the code specifically related to the x410, inlcuding the QSFP ports connections and abstractions

When going through the repository, a very, *very* useful command is `grep -rn --exclude-dir=*build* [search_term]`. This will search recursively through the current directory in all files, excluding build files, for any references. This allows you to more easily 'unbox' the module/interface stack without having to use GitHub's pretty annoying search (and you can't search through forked repositories).

## Common Sources

These might be referred to throughout the doc, and super useful for general information.

[USRP Hardware Drive and USRP Manual](https://files.ettus.com/manual/index.html)

[UHD GitHub Source Code](https://github.com/EttusResearch/uhd)

[RFNoC Specification](https://files.ettus.com/app_notes/RFNoC_Specification.pdf)
