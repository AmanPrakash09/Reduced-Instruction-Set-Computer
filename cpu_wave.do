onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /cpu_tb/sim_write_data
add wave -noupdate /cpu_tb/sim_reset
add wave -noupdate /cpu_tb/sim_read_data
add wave -noupdate /cpu_tb/sim_mem_cmd
add wave -noupdate /cpu_tb/sim_mem_addr
add wave -noupdate /cpu_tb/sim_clk
add wave -noupdate /cpu_tb/sim_Z
add wave -noupdate /cpu_tb/sim_V
add wave -noupdate /cpu_tb/sim_N
add wave -noupdate /cpu_tb/err
add wave -noupdate /cpu_tb/Z
add wave -noupdate /cpu_tb/V
add wave -noupdate /cpu_tb/N
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {53 ps} 0}
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
