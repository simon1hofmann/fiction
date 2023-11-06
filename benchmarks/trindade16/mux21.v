// Benchmark "top" written by ABC on Mon Oct 16 12:28:33 2023

module top (
    pi0, pi1,
    po0, po1  );
  input  pi0, pi1;
  output po0, po1;
  assign new = pi0 & pi1;
  assign po0 = new;
  assign po1 = new;
endmodule
