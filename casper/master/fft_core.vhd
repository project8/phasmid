library IEEE;
use IEEE.std_logic_1164.all;

entity fft_core is
  port (
    ce_1: in std_logic; 
    clk_1: in std_logic; 
    din0_c: in std_logic_vector(49 downto 0); 
    din1_c: in std_logic_vector(49 downto 0); 
    shift: in std_logic_vector(12 downto 0); 
    sync_in: in std_logic; 
    dout0_c: out std_logic_vector(49 downto 0); 
    dout1_c: out std_logic_vector(49 downto 0); 
    fft_of: out std_logic; 
    sync_out: out std_logic
  );
end fft_core;

architecture structural of fft_core is
begin
end structural;

