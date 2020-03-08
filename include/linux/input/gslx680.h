#ifndef _LINUX_GLSX680_H
#define _LINUX_GLSX680_H


struct gslx680_platform_data {
	int irq_pin;
	int reset_pin;

	/* startup defaults for operational parameters */
	bool use_parameters;
	u8 gain;
	u8 threshold;
	u8 offset;
	u8 report_rate;
};

#endif /* _LINUX_GLSX680_H */
