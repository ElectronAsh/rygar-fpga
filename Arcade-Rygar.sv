// Copyright (c) 2019 Josh Bassett
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

module emu
(
  //Master input clock
  input         CLK_50M,

  //Async reset from top-level module.
  //Can be used as initial reset.
  input         RESET,

  //Must be passed to hps_io module
  inout  [45:0] HPS_BUS,

  //Base video clock. Usually equals to CLK_SYS.
  output        CLK_VIDEO,

  //Multiple resolutions are supported using different CE_PIXEL rates.
  //Must be based on CLK_VIDEO
  output        CE_PIXEL,

  //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
  output  [7:0] VIDEO_ARX,
  output  [7:0] VIDEO_ARY,

  output  [7:0] VGA_R,
  output  [7:0] VGA_G,
  output  [7:0] VGA_B,
  output        VGA_HS,
  output        VGA_VS,
  output        VGA_DE,    // = ~(VBlank | HBlank)
  output        VGA_F1,
  output  [1:0] VGA_SL,

  output        LED_USER,  // 1 - ON, 0 - OFF.

  // b[1]: 0 - LED status is system status OR'd with b[0]
  //       1 - LED status is controled solely by b[0]
  // hint: supply 2'b00 to let the system control the LED.
  output  [1:0] LED_POWER,
  output  [1:0] LED_DISK,

  output [15:0] AUDIO_L,
  output [15:0] AUDIO_R,
  output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
  output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

  //ADC
  inout   [3:0] ADC_BUS,

  // SD-SPI
  output        SD_SCK,
  output        SD_MOSI,
  input         SD_MISO,
  output        SD_CS,
  input         SD_CD,

  //High latency DDR3 RAM interface
  //Use for non-critical time purposes
  output        DDRAM_CLK,
  input         DDRAM_BUSY,
  output  [7:0] DDRAM_BURSTCNT,
  output [28:0] DDRAM_ADDR,
  input  [63:0] DDRAM_DOUT,
  input         DDRAM_DOUT_READY,
  output        DDRAM_RD,
  output [63:0] DDRAM_DIN,
  output  [7:0] DDRAM_BE,
  output        DDRAM_WE,

  //SDRAM interface with lower latency
  output        SDRAM_CLK,
  output        SDRAM_CKE,
  output [12:0] SDRAM_A,
  output  [1:0] SDRAM_BA,
  inout  [15:0] SDRAM_DQ,
  output        SDRAM_DQML,
  output        SDRAM_DQMH,
  output        SDRAM_nCS,
  output        SDRAM_nCAS,
  output        SDRAM_nRAS,
  output        SDRAM_nWE,

  input         UART_CTS,
  output        UART_RTS,
  input         UART_RXD,
  output        UART_TXD,
  output        UART_DTR,
  input         UART_DSR,

  // Open-drain User port.
  // 0 - D+/RX
  // 1 - D-/TX
  // 2..5 - USR1..USR4
  // Set USER_OUT to 1 to read from USER_IN.
  input   [5:0] USER_IN,
  output  [5:0] USER_OUT,

  input         OSD_STATUS
);

assign USER_OUT = '1;
assign VGA_F1 = 0;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

//assign AUDIO_R   = AUDIO_L;
assign AUDIO_S   = 1;

assign LED_USER  = ioctl_download;
assign LED_DISK  = 0;
assign LED_POWER = 0;

assign CLK_VIDEO = clk_sys;
assign VIDEO_ARX = 8'd4;
assign VIDEO_ARY = 8'd3;

`include "build_id.v"
localparam CONF_STR = {
  "A.Rygar;;",
  "F,rom;",
  "-;",
  "O1,Aspect Ratio,Original,Wide;",
  "O35,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
  "-;",
  "O89,Lives,3,4,5,2;",
  "OA,Cabinet,Upright,Cocktail;",
  "OBC,Bonus Life,50K 200K 500K,100K 300K 600K,200K 500K,100K;",
  "ODE,Difficulty,Easy,Normal,Hard,Hardest;",
  "OF,Allow Continue,Yes,No;",
  "-;",
  "R0,Reset;",
  "J1,Jump,Fire,Start 1P,Start 2P,Coin;",
  "V,v",`BUILD_DATE
};

////////////////////   CLOCKS   ///////////////////

wire locked;
wire clk_sys;
wire cen_12;

pll pll
(
  .refclk(CLK_50M),
  .outclk_0(clk_sys),
  .outclk_1(SDRAM_CLK),
  .outclk_2(snd_clk),
  .locked(locked)
);

///////////////////////////////////////////////////

wire [31:0] status;
wire  [1:0] buttons;

wire [24:0] ioctl_addr;
wire  [7:0] ioctl_data;
wire        ioctl_wr;
wire        ioctl_download;

wire [10:0] ps2_key;

wire  [8:0] joystick_0, joystick_1;
wire [15:0] joy = joystick_0 | joystick_1;

wire forced_scandoubler;

hps_io #(.STRLEN($size(CONF_STR)>>3)) hps_io
(
  .clk_sys(clk_sys),
  .HPS_BUS(HPS_BUS),

  .conf_str(CONF_STR),

  .buttons(buttons),
  .status(status),
  .forced_scandoubler(forced_scandoubler),

  .ioctl_addr(ioctl_addr),
  .ioctl_dout(ioctl_data),
  .ioctl_wr(ioctl_wr),
  .ioctl_download(ioctl_download),

  .joystick_0(joystick_0),
  .joystick_1(joystick_1),
  .ps2_key(ps2_key)
);

///////////////////////////////////////////////////////////////////

wire [3:0] R, G, B;
wire HSync, VSync, HBlank, VBlank;
wire [2:0] scale = status[5:3];
wire scandoubler = (scale || forced_scandoubler);

video_mixer #(.LINE_LENGTH(256), .HALF_DEPTH(1)) video_mixer
(
  .*,

  .clk_sys(clk_sys),
  .ce_pix(cen_12),
  .ce_pix_out(CE_PIXEL),

  .scanlines(0),
  .scandoubler(scandoubler),
  .hq2x(scale==1),
  .mono(0)
);

wire [22:0] sdram_addr;
wire [31:0] sdram_din;
wire [31:0] sdram_dout;
wire sdram_we;
wire sdram_req;
wire sdram_ack;
wire sdram_ready;
wire sdram_valid;

sdram #(.CLK_FREQ(48.0)) sdram
(
  .reset(~locked),
  .clk(clk_sys),

  // controller interface
  .addr(sdram_addr),
  .din(sdram_din),
  .dout(sdram_dout),
  .we(sdram_we),
  .req(sdram_req),
  .ack(sdram_ack),
  .ready(sdram_ready),
  .valid(sdram_valid),

  // SDRAM interface
  .sdram_a(SDRAM_A),
  .sdram_ba(SDRAM_BA),
  .sdram_dq(SDRAM_DQ),
  .sdram_cke(SDRAM_CKE),
  .sdram_cs_n(SDRAM_nCS),
  .sdram_ras_n(SDRAM_nRAS),
  .sdram_cas_n(SDRAM_nCAS),
  .sdram_we_n(SDRAM_nWE),
  .sdram_dqml(SDRAM_DQML),
  .sdram_dqmh(SDRAM_DQMH)
);

wire       pressed = ps2_key[9];
wire [7:0] code    = ps2_key[7:0];

reg key_left    = 0;
reg key_right   = 0;
reg key_down    = 0;
reg key_up      = 0;
reg key_jump    = 0;
reg key_fire    = 0;
reg key_start_1 = 0;
reg key_start_2 = 0;
reg key_coin    = 0;

always @(posedge clk_sys) begin
  reg old_state;
  old_state <= ps2_key[10];

  if (old_state != ps2_key[10]) begin
    case (code)
      'h75: key_up      <= pressed; // up
      'h72: key_down    <= pressed; // down
      'h6B: key_left    <= pressed; // left
      'h74: key_right   <= pressed; // right
      'h16: key_start_1 <= pressed; // 1
      'h1E: key_start_2 <= pressed; // 2
      'h2E: key_coin    <= pressed; // 5
      'h14: key_fire    <= pressed; // ctrl
      'h11: key_jump    <= pressed; // alt
    endcase
  end
end

wire up      = key_up      | joy[3];
wire down    = key_down    | joy[2];
wire left    = key_left    | joy[1];
wire right   = key_right   | joy[0];
wire jump    = key_jump    | joy[4];
wire fire    = key_fire    | joy[5];
wire start_1 = key_start_1 | joy[6];
wire start_2 = key_start_2 | joy[7];
wire coin    = key_coin    | joy[8];

(*keep*) wire core_reset = RESET | ioctl_download | status[0] | buttons[1];

game game
(
  .reset(core_reset),
  .clk(clk_sys),
  .cen_12(cen_12),

  .joystick_1({2'b0, jump, fire, up, down, right, left}),
  .joystick_2({2'b0, jump, fire, up, down, right, left}),
  .start_1(start_1),
  .start_2(start_2),
  .coin_1(coin),
  .coin_2(1'b0),

  .dip_allow_continue(~status[15]),
  .dip_bonus_life(status[12:11]),
  .dip_cabinet(~status[10]),
  .dip_difficulty(status[14:13]),
  .dip_lives(status[9:8]),

  .sdram_addr(sdram_addr),
  .sdram_din(sdram_din),
  .sdram_dout(sdram_dout),
  .sdram_we(sdram_we),
  .sdram_req(sdram_req),
  .sdram_ack(sdram_ack),
  .sdram_ready(sdram_ready),
  .sdram_valid(sdram_valid),

  .ioctl_addr(ioctl_addr),
  .ioctl_data(ioctl_data),
  .ioctl_wr(ioctl_wr),
  .ioctl_download(ioctl_download),

  .hsync(HSync),
  .vsync(VSync),
  .hblank(HBlank),
  .vblank(VBlank),

  .r(R),
  .g(G),
  .b(B)
);


wire [15:0] snd_z80_addr;
wire [7:0] snd_z80_di;
wire [7:0] snd_z80_do;

wire snd_z80_int_n = opl_irq_n;

wire snd_z80_nmi_n;

wire snd_z80_m1_n;
wire snd_z80_mreq_n;
wire snd_z80_iorq_n;
wire snd_z80_rd_n;
wire snd_z80_wr_n;
wire snd_z80_rfsh_n;
wire snd_z80_halt_n;
wire snd_z80_busak_n;

T80s T80s_inst
(
	.RESET_n(!core_reset) ,		// input  RESET_n
	.CLK(snd_clk) ,				// input  CLK
	.CEN(1'b1) ,					// input  CEN
	
	.WAIT_n(1'b1) ,			 	// input  WAIT_n
	.INT_n( snd_z80_int_n ) ,	// input  INT_n
	.NMI_n(snd_z80_nmi_n) ,		// input  NMI_n
	.BUSRQ_n(1'b1) ,			 	// input  BUSRQ_n

	.M1_n(snd_z80_m1_n) ,		// output  M1_n
	.MREQ_n(snd_z80_mreq_n) ,	// output  MREQ_n
	.IORQ_n(snd_z80_iorq_n) ,	// output  IORQ_n
	.RD_n(snd_z80_rd_n) ,		// output  RD_n
	.WR_n(snd_z80_wr_n) ,		// output  WR_n
	.RFSH_n(snd_z80_rfsh_n) ,	// output  RFSH_n
	.HALT_n(snd_z80_halt_n) ,	// output  HALT_n
	.BUSAK_n(snd_z80_busak_n) ,// output  BUSAK_n

	.OUT0(1'b0) ,					// input  OUT0

	.A(snd_z80_addr) ,			// output [15:0] A

	.DI(snd_z80_di) ,				// input [7:0] DI
	.DO(snd_z80_do) 				// output [7:0] DO
);


(*keep*) wire snd_rom_cs = !snd_z80_mreq_n && (snd_z80_addr>=16'h0000 && snd_z80_addr<=16'h3fff);
(*keep*) wire snd_ram_cs = !snd_z80_mreq_n && (snd_z80_addr>=16'h4000 && snd_z80_addr<=16'h7fff);
(*keep*) wire snd_opl_cs = !snd_z80_mreq_n && (snd_z80_addr>=16'h8000 && snd_z80_addr<=16'hbfff);

// The sound latch (chip "2J" on schematic PDF page 9) can be read within this whole address range...
//(*keep*) wire snd_lat_cs			= !snd_z80_mreq_n && (snd_z80_addr>=16'hC000 && snd_z80_addr<=16'hFFFF);
(*keep*) wire snd_lat_cs			= !snd_z80_mreq_n && (snd_z80_addr>=16'hC000 && snd_z80_addr<=16'hC001);	// TESTING !!

// These addresses are only used during WRITES...
(*keep*) wire snd_adpcm_start_cs	= !snd_z80_mreq_n && (snd_z80_addr>=16'hC000 && snd_z80_addr<=16'hCFFF);
(*keep*) wire snd_adpcm_end_cs	= !snd_z80_mreq_n && (snd_z80_addr>=16'hD000 && snd_z80_addr<=16'hDFFF);
(*keep*) wire snd_volume_cs 		= !snd_z80_mreq_n && (snd_z80_addr>=16'hE000 && snd_z80_addr<=16'hEFFF);

//(*keep*) wire snd_req_clear_cs	= !snd_z80_mreq_n && (snd_z80_addr>=16'hF000 && snd_z80_addr<=16'hFFFF);
(*keep*) wire snd_req_clear_cs	= !snd_z80_mreq_n && (snd_z80_addr>=16'hF000 && snd_z80_addr<=16'hF001);	// TESTING !!


assign snd_z80_di = (snd_rom_cs) ? snd_rom_do :
						  (snd_ram_cs) ? snd_ram_do :
						  (snd_opl_cs) ?	   opl_do :
						  (snd_lat_cs) ?  snd_latch :
													 8'hFF;

(*noprune*) reg req_flag;

// This reg represents counters 2F, 2E, 2H...
(*noprune*) reg [15:0] snd_adpcm_addr_reg;

// This reg represents the '273 latch 3F...
(*noprune*) reg [7:0] snd_adpcm_end_reg;

// This reg represents the '174 latch 3C...
(*noprune*) reg [3:0] snd_volume_reg;


// The Q_N output of '74 "Request" flip flop 6C is tied directly to the NMI_N input of the Sound Z80...
assign snd_z80_nmi_n = !req_flag;

(*noprune*) reg [7:0] snd_latch;
always @(posedge snd_clk or posedge core_reset)
if (core_reset) begin
	//req_flag <= 1'b0;		// The request flip flop gets CLEARED by a /RESET.
	//snd_latch <= 8'h00;

	req_flag <= 1'b1;			// TESTING !! Assert the request flag at reset.
	//snd_latch <= 8'h32;		// TESTING !! Play the Rygar intro music. ElectronAsh.
	snd_latch <= 8'h34;		// TESTING !! Play the Rygar first level music. ElectronAsh.
end
else begin
	// NOTE: The Rygar board doesn't appear to use this method of clearing the REQ / NMI flag.
	// This is just for testing atm. ElectronAsh.
	if (!snd_z80_m1_n && !snd_z80_iorq_n) begin
		req_flag <= 1'b0;	// When the Z80 acknowledges the NMI, de-assert the REQ / NMI signal!
	end

	// A write to the ADPCM start address will clear the lower 8 bits of the address.
	// Sound Z80 DATA bits [7:0] are then used for the upper 8 bits of the ADPCM address...
	if (snd_adpcm_start_cs && !snd_z80_wr_n) snd_adpcm_addr_reg <= {snd_z80_do[7:0], 8'h00};
	
	// A write to the ADPCM start address loads the '273 latch at 3F.
	// That value is then compared to the upper 8 bits ([15:9]) of the ADPCM address counter.
	// When the ADPCM ADDR counter value matches the END address, I *think* it inhibits the address counter outputs,
	// and disables the ADPCM output via the volume later, and resets the lower 8 bits of the ADPCM address counter.
	if (snd_adpcm_end_cs && !snd_z80_wr_n) snd_adpcm_end_reg <= snd_z80_do;

	// A write to the volume reg just latches the lower four data bits from the Z80, AFAIK...
	if (snd_volume_cs && !snd_z80_wr_n) snd_volume_reg <= snd_z80_do[3:0];

	// A write to "REQ" flag address range will CLEAR the '74 flip flop at 6C.
	// The Q output of that flip can be read by the MAIN Z80 via one of the input ports.
	// The Q_N output is tied directly to the NMI_N input of the Sound Z80...
	if (snd_req_clear_cs && !snd_z80_wr_n) begin
		req_flag <= 1'b0;
		snd_latch <= 8'h00;	// TESTING !!
	end
end


													
wire [7:0] snd_rom_do;
sound_rom	sound_rom_inst (
	.clock ( snd_clk ),
	.address ( snd_z80_addr[13:0] ),
	.q ( snd_rom_do )
);



wire [7:0] snd_ram_di = snd_z80_do;
wire snd_ram_wren = snd_ram_cs && !snd_z80_mreq_n && !snd_z80_wr_n;

wire [7:0] snd_ram_do;
sound_ram	sound_ram_inst (
	.clock ( snd_clk ),
	.address ( snd_z80_addr[10:0] ),
	.data ( snd_ram_di ),
	.wren ( snd_ram_wren ),
	.q ( snd_ram_do )
);


wire opl_irq_n;
wire [1:0] opl_addr = {1'b0, snd_z80_addr[0]};	// Check!
wire [7:0] opl_di = snd_z80_do;
wire [7:0] opl_do;
wire opl_we = snd_opl_cs && !snd_z80_wr_n;

wire [15:0] opl_samp_l;
wire [15:0] opl_samp_r;

opl3 opl3_inst
(
	.clk(snd_clk) ,				// input  clk
	
	.clk_opl(clk_sys) ,			// input  clk_opl
	
	.rst_n(!core_reset) ,		// input  rst_n
	
	.irq_n( opl_irq_n ) ,		// output  irq_n
	
	.period_80us( 13'd2560 ) ,	// input [12:0] period_80us
	
	.addr(opl_addr) ,				// input [1:0] addr
	.din(opl_di) ,					// input [7:0] din
	.dout(opl_do) ,				// output [7:0] dout
	.we(opl_we) ,					// input  we
	
	.sample_l(opl_samp_l) ,		// output [15:0] sample_l
	.sample_r(opl_samp_r) 		// output [15:0] sample_r
);

assign AUDIO_L = opl_samp_l;
assign AUDIO_R = opl_samp_r;


endmodule
