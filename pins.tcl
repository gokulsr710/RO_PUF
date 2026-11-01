# =============================================================================
# pins.tcl - pin assignments for top_ov7670_vga on the DE2-115 (EP4CE115F29C7)
#
# Usage (either way):
#   A) Quartus GUI:  Tools > Tcl Scripts... > select pins.tcl > Run
#      (or in the Tcl console:  source pins.tcl)
#   B) Command line, from the project folder:
#        quartus_sh -t pins.tcl
#
# Pin numbers are from Terasic's DE2-115 golden pin list.
# Camera on GPIO header: 0 SIOC, 1 SIOD, 2 VSYNC, 3 HREF, 4 PCLK, 5 XCLK,
#                        6..13 D0..D7, 14 RESET, 15 PWDN
# =============================================================================

package require ::quartus::project

set PROJECT_NAME top_ov7670_vga
set opened_here 0

if {![is_project_open]} {
    if {[project_exists $PROJECT_NAME]} {
        project_open $PROJECT_NAME
    } else {
        project_new $PROJECT_NAME
    }
    set opened_here 1
}

proc assign_pin {name pin std} {
    set_location_assignment PIN_$pin -to $name
    set_instance_assignment -name IO_STANDARD $std -to $name
}

set STD33 "3.3-V LVTTL"
set STD25 "2.5 V"

# ---- Clock -------------------------------------------------------------------
foreach {n p} {
    CLOCK_50 Y2
} { assign_pin $n $p $STD33 }

# ---- Push buttons (2.5 V bank) ----------------------------------------------
foreach {n p} {
    KEY[0] M23
    KEY[1] M21
    KEY[2] N21
    KEY[3] R24
} { assign_pin $n $p $STD25 }

# ---- Slide switches (2.5 V bank) --------------------------------------------
foreach {n p} {
    SW[0] AB28
    SW[1] AC28
    SW[2] AC27
    SW[3] AD27
    SW[4] AB27
    SW[5] AC26
    SW[6] AD26
    SW[7] AB26
    SW[8] AC25
    SW[9] AB25
    SW[10] AC24
    SW[11] AB24
    SW[12] AB23
    SW[13] AA24
    SW[14] AA23
    SW[15] AA22
    SW[16] Y24
    SW[17] Y23
} { assign_pin $n $p $STD25 }

# ---- Green LEDs (2.5 V bank) -------------------------------------------------
foreach {n p} {
    LEDG[0] E21
    LEDG[1] E22
    LEDG[2] E25
    LEDG[3] E24
    LEDG[4] H21
    LEDG[5] G20
    LEDG[6] G22
    LEDG[7] G21
    LEDG[8] F17
} { assign_pin $n $p $STD25 }

# ---- VGA (ADV7123 DAC) -------------------------------------------------------
foreach {n p} {
    VGA_R[0] E12
    VGA_R[1] E11
    VGA_R[2] D10
    VGA_R[3] F12
    VGA_R[4] G10
    VGA_R[5] J12
    VGA_R[6] H8
    VGA_R[7] H10
    VGA_G[0] G8
    VGA_G[1] G11
    VGA_G[2] F8
    VGA_G[3] H12
    VGA_G[4] C8
    VGA_G[5] B8
    VGA_G[6] F10
    VGA_G[7] C9
    VGA_B[0] B10
    VGA_B[1] A10
    VGA_B[2] C11
    VGA_B[3] B11
    VGA_B[4] A11
    VGA_B[5] C12
    VGA_B[6] D11
    VGA_B[7] D12
    VGA_BLANK_N F11
    VGA_CLK A12
    VGA_HS G13
    VGA_SYNC_N C10
    VGA_VS C13
} { assign_pin $n $p $STD33 }

# ---- GPIO header (camera) - keep JP6 on 3.3 V -------------------------------
foreach {n p} {
    GPIO[0] AB22
    GPIO[1] AC15
    GPIO[2] AB21
    GPIO[3] Y17
    GPIO[4] AC21
    GPIO[5] Y16
    GPIO[6] AD21
    GPIO[7] AE16
    GPIO[8] AD15
    GPIO[9] AE15
    GPIO[10] AC19
    GPIO[11] AF16
    GPIO[12] AD19
    GPIO[13] AF15
    GPIO[14] AF24
    GPIO[15] AE21
    GPIO[16] AF25
    GPIO[17] AC22
    GPIO[18] AE22
    GPIO[19] AF21
    GPIO[20] AF22
    GPIO[21] AD22
    GPIO[22] AG25
    GPIO[23] AD25
    GPIO[24] AH25
    GPIO[25] AE25
    GPIO[26] AG22
    GPIO[27] AE24
    GPIO[28] AH22
    GPIO[29] AF26
    GPIO[30] AE20
    GPIO[31] AG23
    GPIO[32] AF20
    GPIO[33] AH26
    GPIO[34] AH23
    GPIO[35] AG26
} { assign_pin $n $p $STD33 }

# ---- Finish ------------------------------------------------------------------
export_assignments
if {$opened_here} {
    project_close
}
puts "Pin assignments applied."
