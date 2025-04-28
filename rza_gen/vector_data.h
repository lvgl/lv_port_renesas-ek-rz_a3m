/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (26)
#endif

/** Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

/* ISR prototypes */
void gtm_int_isr(IRQn_Type const irq);
void lcdc_vspd_int(IRQn_Type const irq);
void mipi_dsi_seq0(IRQn_Type const irq);
void mipi_dsi_seq1(IRQn_Type const irq);
void mipi_dsi_vin1(IRQn_Type const irq);
void mipi_dsi_rcv(IRQn_Type const irq);
void mipi_dsi_ferr(IRQn_Type const irq);
void mipi_dsi_ppi(IRQn_Type const irq);
void riic_master_rxi_isr(IRQn_Type const irq);
void riic_master_txi_isr(IRQn_Type const irq);
void riic_master_tei_isr(IRQn_Type const irq);
void riic_master_naki_isr(IRQn_Type const irq);
void riic_master_spi_isr(IRQn_Type const irq);
void riic_master_sti_isr(IRQn_Type const irq);
void riic_master_ali_isr(IRQn_Type const irq);
void riic_master_tmoi_isr(IRQn_Type const irq);
void ssi_txi_isr(IRQn_Type const irq);
void ssi_rxi_isr(IRQn_Type const irq);
void ssi_int_isr(IRQn_Type const irq);
void r_intc_irq_isr(IRQn_Type const irq);
void scif_uart_eri_isr(IRQn_Type const irq);
void scif_uart_bri_isr(IRQn_Type const irq);
void scif_uart_rxi_isr(IRQn_Type const irq);
void scif_uart_txi_isr(IRQn_Type const irq);
void scif_uart_tei_isr(IRQn_Type const irq);

/** Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif /* VECTOR_DATA_H */
