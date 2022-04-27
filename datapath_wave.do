onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /datapath_tb/err
add wave -noupdate /datapath_tb/sim_ALUop
add wave -noupdate /datapath_tb/sim_N
add wave -noupdate /datapath_tb/sim_V
add wave -noupdate /datapath_tb/sim_Z
add wave -noupdate /datapath_tb/sim_asel
add wave -noupdate /datapath_tb/sim_bsel
add wave -noupdate /datapath_tb/sim_clk
add wave -noupdate /datapath_tb/sim_datapath_out
add wave -noupdate /datapath_tb/sim_loada
add wave -noupdate /datapath_tb/sim_loadb
add wave -noupdate /datapath_tb/sim_loadc
add wave -noupdate /datapath_tb/sim_loads
add wave -noupdate /datapath_tb/sim_mdata
add wave -noupdate /datapath_tb/sim_readnum
add wave -noupdate /datapath_tb/sim_shift
add wave -noupdate /datapath_tb/sim_sximm5
add wave -noupdate /datapath_tb/sim_sximm8
add wave -noupdate /datapath_tb/sim_vsel
add wave -noupdate /datapath_tb/sim_write
add wave -noupdate /datapath_tb/sim_writenum
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {100 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
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
WaveRestoreZoom {0 ps} {1 ns}
