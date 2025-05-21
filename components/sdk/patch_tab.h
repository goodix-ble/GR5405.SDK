#ifndef __PATCH_TAB_H_
#define __PATCH_TAB_H_

#include <stdint.h>
#include <stdio.h>

typedef int (*ke_msg_func_t)(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

struct ke_msg_handler
{
    /// Id of the handled message.
    ke_msg_id_t id;
    /// Pointer to the handler function for the msgid above.
    ke_msg_func_t func;
};

typedef struct hci_cmd_handler_tab_info
{
    struct ke_msg_handler * hci_handler_tab_p;
    uint32_t tab_size;
} hci_cmd_handler_tab_info_t;

typedef int (*llm_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);

typedef int (*gapm_hci_evt_hdl_func_t)(uint16_t opcode, void const *param);

struct gapm_hci_evt_handler
{
    uint16_t opcode;
    gapm_hci_evt_hdl_func_t func;
};

typedef struct
{
    struct gapm_hci_evt_handler *gapm_hci_evt_handler_p;
    uint32_t tab_size;
} gapm_hci_evt_handler_tab_info_t;

typedef struct
{
    ke_msg_func_t ori_func_addr;
    ke_msg_func_t new_func_addr;
} msg_tab_item_t;

typedef struct
{
    llm_hci_cmd_hdl_func_t ori_func_addr;
    llm_hci_cmd_hdl_func_t new_func_addr;
} llm_hci_cmd_tab_item_t;

typedef struct
{
    gapm_hci_evt_hdl_func_t ori_func_addr;
    gapm_hci_evt_hdl_func_t new_func_addr;
} gapm_hci_evt_tab_item_t;


/// LLD connection offset update indication structure
/*@TRACE*/
struct lld_con_offset_upd_ind
{
    /// Connection offset in half-slots (312.5 us)
    uint16_t con_offset;
};

extern int hci_command_llm_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id,ke_task_id_t const src_id);

// gapm cfg
extern int gapm_set_dev_config_cmd_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

extern int gapm_hci_handler_patch(ke_msg_id_t const msgid, void const* event,
    ke_task_id_t dest_id, ke_task_id_t opcode);

#if (CFG_MUL_LINK_WITH_SAME_DEV)
// gapm hci event for multiple link
extern int hci_le_adv_set_term_evt_handler_patch(uint16_t opcode, void const *p_event);

// gapc task for multiple link
extern int gapc_bond_cfm_handler_patch(ke_msg_id_t const msgid, void *cfm,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

extern int lld_adv_end_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif

#if (CFG_CCC_SC_OOB_PAIR_SUPPORT)
extern int llm_pub_key_gen_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif

extern int llc_stopped_ind_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

extern int hci_command_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

extern int lld_con_offset_upd_ind_handler_patch(ke_msg_id_t const msgid, struct lld_con_offset_upd_ind *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

#if (CFG_MAX_ADVS)
extern int gapm_set_adv_data_cmd_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

extern int lld_adv_end_ind_handler_patch2(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif

extern int lld_llcp_rx_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

//!!!!!NOTE: this file should not be included by other files but ble.c!!!!!!
msg_tab_item_t msg_tab[] =
{
    {(ke_msg_func_t)0x000105a1, (ke_msg_func_t)gapm_set_dev_config_cmd_handler_patch},
    {(ke_msg_func_t)0x00011f3d, (ke_msg_func_t)hci_command_llm_handler_patch},
    {(ke_msg_func_t)0x0000f22d, (ke_msg_func_t)gapm_hci_handler_patch},

    #if (CFG_MAX_ADVS)
    {(ke_msg_func_t)0x00010519, (ke_msg_func_t)gapm_set_adv_data_cmd_handler_patch},
    #if (CFG_MUL_LINK_WITH_SAME_DEV)
    {(ke_msg_func_t)0x0000a1e9, (ke_msg_func_t)gapc_bond_cfm_handler_patch},
    {(ke_msg_func_t)0x0001f829, (ke_msg_func_t)lld_adv_end_ind_handler_patch},
    #else
    {(ke_msg_func_t)0x0001f829, (ke_msg_func_t)lld_adv_end_ind_handler_patch2},
    #endif
    #endif

    #if (CFG_CCC_SC_OOB_PAIR_SUPPORT)
    {(ke_msg_func_t)0x0002ddb5, (ke_msg_func_t)llm_pub_key_gen_ind_handler_patch},
    #endif


    {(ke_msg_func_t)0x00011e95, (ke_msg_func_t)hci_command_handler_patch},

    {(ke_msg_func_t)0x00024cd5, (ke_msg_func_t)lld_con_offset_upd_ind_handler_patch},
    {(ke_msg_func_t)0x000286dd, (ke_msg_func_t)lld_llcp_rx_ind_handler_patch},
};

extern int hci_le_add_dev_to_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_rmv_dev_from_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_clear_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_set_addr_resol_en_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_set_priv_mode_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_set_ext_adv_param_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_ext_create_con_cmd_handler_patch(void const *param, uint16_t opcode);

#if CFG_MAX_ADVS
extern int hci_le_set_ext_scan_rsp_data_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_set_ext_adv_en_cmd_handler_patch(void const *param, uint16_t opcode);
#endif

#if CFG_SC_PAIR_SUPPORT
extern int hci_le_rd_local_p256_public_key_cmd_handler_patch(void const *param, uint16_t opcode);
#endif

llm_hci_cmd_tab_item_t llm_hci_cmd_tab[] =
{
     // hci cmd for common
    {(llm_hci_cmd_hdl_func_t)0x000126fd, (llm_hci_cmd_hdl_func_t)hci_le_add_dev_to_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00014ced, (llm_hci_cmd_hdl_func_t)hci_le_rmv_dev_from_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00012b1d, (llm_hci_cmd_hdl_func_t)hci_le_clear_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00014e21, (llm_hci_cmd_hdl_func_t)hci_le_set_addr_resol_en_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00016355, (llm_hci_cmd_hdl_func_t)hci_le_set_priv_mode_cmd_handler_patch},

    #if CFG_MAX_ADVS
    {(llm_hci_cmd_hdl_func_t)0x00015f9d, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_scan_rsp_data_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x0001573d, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_adv_en_cmd_handler_patch},
    #endif

    #if CFG_MASTER_SUPPORT
    {(llm_hci_cmd_hdl_func_t)0x00013e9d, (llm_hci_cmd_hdl_func_t)hci_le_ext_create_con_cmd_handler_patch},
    #endif

    #if CFG_SUPER_ADV_SUPPORT
    {(llm_hci_cmd_hdl_func_t)0x000159e9, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_adv_param_cmd_handler_patch},
    #endif

    #if (CFG_SC_PAIR_SUPPORT)
    {(llm_hci_cmd_hdl_func_t)0x00014641, (llm_hci_cmd_hdl_func_t)hci_le_rd_local_p256_public_key_cmd_handler_patch},
    #endif
};

gapm_hci_evt_tab_item_t gapm_hci_evt_tab[] =
{
    {NULL, NULL},

    #if (CFG_MUL_LINK_WITH_SAME_DEV)
    {(gapm_hci_evt_hdl_func_t)0x00012991, (gapm_hci_evt_hdl_func_t)hci_le_adv_set_term_evt_handler_patch},
    #endif
};

#endif  // __PATCH_TAB_H_
