#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

struct sys_timer;

extern void timer_init(int irq);
extern void mmp2_clear_pmic_int(void);

extern struct sys_timer pxa168_timer;
extern struct sys_timer pxa910_timer;
extern struct sys_timer mmp2_timer;
extern void __init pxa168_init_irq(void);
extern void __init pxa910_init_irq(void);
extern void __init mmp2_init_icu(void);
extern void __init mmp2_init_irq(void);

extern void __init icu_init_irq(void);
extern void __init pxa_map_io(void);
#define REG32(x)       (*(volatile unsigned long *)(x))
#define RIPC0_STATUS   REG32(0xfe03D000)
extern void release_RIPC(void);
extern void get_RIPC(void);
extern struct mbus_dram_target_info pxa168_mbus_dram_info;
extern void pxa168_setup_cpu_mbus(void);
