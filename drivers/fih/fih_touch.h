#ifndef __FIH_TOUCH_H
#define __FIH_TOUCH_H

void fih_info_set_touch(char *info);
enum vendor_id
{
    LGD = 1,
    JDI = 2,
};
//SW8-DH-Double_Tap-00+[
enum
{
    FIH_NONE = 0,
    FIH_DOUBLE_TAP,
    FIH_PASSWORD,
};
enum
{
    FIH_PROX_NEAR = 0,
    FIH_PROX_FAR,
};
//SW8-DH-Double_Tap-00+]


struct fih_touch_cb
{
    void (*touch_selftest)(void);
    int (*touch_selftest_result)(void);
    void (*touch_tpfwver_read)(char *);
    void (*touch_tpfwimver_read)(char *);
    void (*touch_fwupgrade)(int);
    void (*touch_fwupgrade_read)(char *);
    int (*touch_fwback_read)(void);
    void (*touch_fwback_write)(void);
    int (*touch_vendor_id_read)(void);
    void (*touch_vendor_read)(char *);
    unsigned int (*touch_double_tap_read)(void);
    int (*touch_double_tap_write)(unsigned int);
    unsigned int (*touch_prox_status_read)(void);
    int (*touch_prox_status_write)(unsigned int);
    void (*touch_alt_rst)(void);
    int (*touch_alt_st_count)(void);
    void (*touch_alt_st_enable)(int);
    void (*touch_side_touch_enable)(unsigned int);
    unsigned int (*touch_side_touch_status)(void);
    int (*touch_unlock_write)(unsigned int);
    unsigned int (*touch_unlock_read)(void);
};

#endif /* __FIH_TOUCH_H */
