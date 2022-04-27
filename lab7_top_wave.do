onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /lab7_check_tb/KEY
add wave -noupdate /lab7_check_tb/SW
add wave -noupdate /lab7_check_tb/LEDR
add wave -noupdate /lab7_check_tb/HEX0
add wave -noupdate /lab7_check_tb/HEX1
add wave -noupdate /lab7_check_tb/HEX2
add wave -noupdate /lab7_check_tb/HEX3
add wave -noupdate /lab7_check_tb/HEX4
add wave -noupdate /lab7_check_tb/HEX5
add wave -noupdate /lab7_check_tb/err
add wave -noupdate /lab7_check_tb/DUT/CPU/SM/state
add wave -noupdate /lab7_check_tb/DUT/read_data
add wave -noupdate /lab7_check_tb/DUT/write_data
add wave -noupdate /lab7_check_tb/DUT/mem_addr
add wave -noupdate /lab7_check_tb/DUT/mem_cmd
add wave -noupdate /lab7_check_tb/DUT/CPU/DP/REGFILE/R0
add wave -noupdate /lab7_check_tb/DUT/CPU/DP/REGFILE/R1
add wave -noupdate /lab7_check_tb/DUT/CPU/DP/REGFILE/R2
add wave -noupdate /lab7_check_tb/DUT/CPU/DP/REGFILE/R3
add wave -noupdate /lab7_check_tb/DUT/CPU/DP/REGFILE/R4
add wave -noupdate /lab7_check_tb/DUT/CPU/DP/REGFILE/R5
add wave -noupdate /lab7_check_tb/DUT/CPU/DP/REGFILE/R6
add wave -noupdate /lab7_check_tb/DUT/CPU/DP/REGFILE/R7
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {120 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 252
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {900 ps}
