`default_nettype none

// Keep I/O fixed for TinyTapeout
module tt_um_haoyang_serv(

  input  wire [7:0] ui_in,	// Dedicated inputs
	output wire [7:0] uo_out,	// Dedicated outputs
	input  wire [7:0] uio_in,	// IOs: Input path
	output wire [7:0] uio_out,	// IOs: Output path
	output wire [7:0] uio_oe,	// IOs: Enable path (active high: 0=input, 1=output)
	input  wire       ena,
	input  wire       clk,
	input  wire       rst_n
  );
  //
  assign uio_oe = 8'hff;

  reg [7:0] tempout = 8'b0;
  assign uo_out[7:4]=tempout[7:4];

 //

  wire serv_clk =   clk; // cpu clock
  wire spi_clk =      ui_in[0]; // spi clock
  wire data =         ui_in[1];
  wire reset =        ui_in[2];
  // if need scan chain, need:
  //  1. data select
  //  2. scan data
  //  3. clk for SC

  wire timer_irq =ui_in[3]; // temp set


  parameter reset_strategy = "MINI";
  parameter sim = 0;
  parameter with_csr = 0;
  parameter [0:0] compress = 0;
  parameter [0:0] align = 0;

  wire [4+with_csr:0] wreg0;
  wire [4+with_csr:0] wreg1;
  wire [4+with_csr:0] rreg0;
  wire [4+with_csr:0] rreg1;
  wire 	      rf_wreq;
  wire 	      rf_rreq;
  wire 	      wen0;
  wire 	      wen1;
  wire 	      wdata0;
  wire 	      wdata1;
  wire 	      rf_ready = ui_in[4]; //temp set
  wire 	      rdata0   = ui_in[5]; //temp set
  wire 	      rdata1   = ui_in[6]; //temp set

  wire [31:0] 	wb_ibus_adr;
  wire [31:0] 	wb_ibus_rdt;
  wire 	wb_ibus_cyc;
  wire 	wb_ibus_ack;

  wire [31:0] 	wb_dbus_adr;
  wire [31:0] 	wb_dbus_dat;
  wire [3:0] 	wb_dbus_sel;
  wire [31:0] 	wb_dbus_rdt;
  wire 	wb_dbus_we;
  wire 	wb_dbus_cyc;
  wire 	wb_dbus_ack;

  wire [31:0] 	wb_dmem_adr;
  wire [31:0] 	wb_dmem_dat;
  wire [3:0] 	wb_dmem_sel;
  wire [31:0] 	wb_dmem_rdt;
  wire 	wb_dmem_we;
  wire 	wb_dmem_cyc;
  wire 	wb_dmem_ack;

  wire [31:0] 	wb_mem_adr;
  wire [31:0] 	wb_mem_dat;
  wire [3:0] 	wb_mem_sel;
  wire [31:0] 	wb_mem_rdt;
  wire 	wb_mem_we;
  wire 	wb_mem_cyc;
  wire 	wb_mem_ack;

  wire 	wb_gpio_dat;
  wire 	wb_gpio_we;
  wire 	wb_gpio_cyc;
  wire 	wb_gpio_rdt;


  servant_arbiter u_arbiter (
    .i_wb_cpu_dbus_adr (wb_dbus_adr),
    .i_wb_cpu_dbus_dat (wb_dbus_dat),
    .i_wb_cpu_dbus_sel (wb_dbus_sel),
    .i_wb_cpu_dbus_we  (wb_dbus_we ),
    .i_wb_cpu_dbus_cyc (wb_dbus_cyc),
    .o_wb_cpu_dbus_rdt (wb_dbus_rdt),
    .o_wb_cpu_dbus_ack (wb_dbus_ack),

    .i_wb_cpu_ibus_adr (wb_ibus_adr),
    .i_wb_cpu_ibus_cyc (wb_ibus_cyc),
    .o_wb_cpu_ibus_rdt (wb_ibus_rdt),
    .o_wb_cpu_ibus_ack (wb_ibus_ack),

    .o_wb_cpu_adr (wb_mem_adr),
    .o_wb_cpu_dat (wb_mem_dat),
    .o_wb_cpu_sel (wb_mem_sel),
    .o_wb_cpu_we  (wb_mem_we ),
    .o_wb_cpu_cyc (wb_mem_cyc),
    .i_wb_cpu_rdt (wb_mem_rdt), // scan chain
    .i_wb_cpu_ack (wb_mem_ack)  // scan chain
  );


  serv_top #(
    .RESET_PC (32'h0000_0000),
    .PRE_REGISTER(1),
    .RESET_STRATEGY (reset_strategy),
    .WITH_CSR (with_csr),
    .COMPRESSED(compress),
    .ALIGN(align))
  cpu
  (
    .clk          (serv_clk),
    .i_rst        (reset),
    .i_timer_irq  (timer_irq),

    .o_rf_rreq    (rf_rreq),
    .o_rf_wreq    (rf_wreq),
    .i_rf_ready   (rf_ready),
    .o_wreg0      (wreg0),
    .o_wreg1      (wreg1),
    .o_wen0       (wen0),
    .o_wen1       (wen1),
    .o_wdata0     (wdata0),
    .o_wdata1     (wdata1),
    .o_rreg0      (rreg0),
    .o_rreg1      (rreg1),
    .i_rdata0     (rdata0),
    .i_rdata1     (rdata1),

    .o_ibus_adr   (wb_ibus_adr),
    .o_ibus_cyc   (wb_ibus_cyc),
    .i_ibus_rdt   (wb_ibus_rdt),
    .i_ibus_ack   (wb_ibus_ack),

    .o_dbus_adr   (wb_dbus_adr),
    .o_dbus_dat   (wb_dbus_dat),
    .o_dbus_sel   (wb_dbus_sel),
    .o_dbus_we    (wb_dbus_we),
    .o_dbus_cyc   (wb_dbus_cyc),
    .i_dbus_rdt   (wb_dbus_rdt),
    .i_dbus_ack   (wb_dbus_ack)
  );


  // scanchain_local #(
  //   .SCAN_LENGTH(96))
  // u_scanchain_local
  // (
  //   // Inputs from TinyTapeout scanchain to our internal scanchain
  //   .clk_in          (clk),
  //   .data_in         (data),
  //   .scan_select_in  (scan_select),

  //   // Pass all signals out from our internal scanchain, only really need data
  //   .clk_out         (uo_out[0]),
  //   .data_out        (uo_out[1]),
  //   .scan_select_out (uo_out[2]),

  //   // data
  //   .module_data_out ({
  //     // Bus interface 96
  //     wb_mem_adr[31:0],   // 32
  //     wb_mem_dat,         // 32
  //     wb_mem_sel,         // 4
  //     wb_mem_we,          // 1
  //     wb_mem_cyc,         // 1
  //     // RF interface
  //     rf_wreq,            // 1
  //     rf_rreq,            // 1
  //     wreg0,              // 5
  //     wreg1,              // 5
  //     wen0,               // 1
  //     wen1,               // 1
  //     wdata0,             // 1
  //     wdata1,             // 1
  //     rreg0,              // 5
  //     rreg1}),            // 5

  //   .module_data_in  ({
  //     // Bus interface
  //     wb_mem_rdt,         // 32
  //     wb_mem_ack,         // 1
  //     timer_irq,          // 1
  //     rf_ready,           // 1
  //     rdata0,             // 1
  //     rdata1})            // 1
  // );

spi_top spibridge( // input from serv or user (clk and reset)
.wb_clk_i   (spi_clk ),
.wb_rst_i   (reset),   // ui_in[2]
.wb_adr_i   (wb_mem_adr),
.wb_dat_i   (wb_mem_dat),
.wb_dat_o   (wb_mem_rdt),// to serv
.wb_sel_i   (wb_mem_sel),
.wb_we_i    (wb_mem_we ),
.wb_stb_i   (1'b1),
.wb_cyc_i   (wb_mem_cyc),
.wb_ack_o   (wb_mem_ack),// to serv
.wb_err_o   (uo_out[0]),// to user
.wb_int_o   (uo_out[1]),// to user
.ss_pad_o   (uio_out),
.sclk_pad_o (uo_out[2]),
.mosi_pad_o (uo_out[3]),
.miso_pad_i (data) // ui_in[1]
);


endmodule
