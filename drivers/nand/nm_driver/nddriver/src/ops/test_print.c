#include "lib_nandopsinterface.h"
#include "ndmessage.h"
#include "nand_debug.h"

void dump_taskmsg_ops(union taskmsghead *ops)
{
    ndd_print(NDD_DEBUG, "/****************************/\n");
    ndd_print(NDD_DEBUG, "type = ");
    switch(ops->bits.type){
        case MSG_MCU_INIT:
            ndd_print(NDD_DEBUG, "MSG_MCU_INIT\n");
            break;
        case MSG_MCU_PREPARE:
            ndd_print(NDD_DEBUG, "MSG_MCU_PREPARE\n");
            break;
        case MSG_MCU_CMD:
            ndd_print(NDD_DEBUG, "MSG_MCU_CMD\n");
            break;
        case MSG_MCU_BADBLOCK:
            ndd_print(NDD_DEBUG, "MSG_MCU_BADBLOCK\n");
            break;
        case MSG_MCU_DATA:
            ndd_print(NDD_DEBUG, "MSG_MCU_DATA\n");
            break;
        case MSG_MCU_RET:
            ndd_print(NDD_DEBUG, "MSG_MCU_RET\n");
            break;
        case MSG_MCU_PARITY:
            ndd_print(NDD_DEBUG, "MSG_MCU_PARITY\n");
            break;
        default:
            ndd_print(NDD_DEBUG, "ERROR!!!!\n");
            break;
    }
    ndd_print(NDD_DEBUG, "model = ");
    switch(ops->bits.model){
        case MCU_NO_RB:
            ndd_print(NDD_DEBUG, "MCU_NO_RB\n");
            break;
        case MCU_WITH_RB:
            ndd_print(NDD_DEBUG, "MCU_WITH_RB\n");
            break;
        case MCU_READ_DATA:
            ndd_print(NDD_DEBUG, "MCU_READ_DATA\n");
            break;
        case MCU_WRITE_DATA_WAIT:
            ndd_print(NDD_DEBUG, "MCU_WRITE_DATA_WAIT\n");
            break;
        case MCU_WRITE_DATA:
            ndd_print(NDD_DEBUG, "MCU_WRITE_DATA\n");
            break;
        case MCU_ISBADBLOCK:
            ndd_print(NDD_DEBUG, "MCU_ISBADBLOCK\n");
            break;
        case MCU_MARKBADBLOCK:
            ndd_print(NDD_DEBUG, "MCU_MARKBADBLOCK\n");
            break;
        default:
            ndd_print(NDD_DEBUG, "ERROR!!!!\n");
            break;
    }
    ndd_print(NDD_DEBUG, "state = ");
    switch(ops->bits.state){
        case CHAN1_DATA:
            ndd_print(NDD_DEBUG, "CHAN1_DATA\n");
            break;
        case CHAN1_PARITY:
            ndd_print(NDD_DEBUG, "CHAN1_PARITY\n");
            break;
        case BCH_PREPARE:
            ndd_print(NDD_DEBUG, "BCH_PREPARE\n");
            break;
        case BCH_FINISH:
            ndd_print(NDD_DEBUG, "BCH_FINISH\n");
            break;
        case CHAN2_DATA:
            ndd_print(NDD_DEBUG, "CHAN2_DATA\n");
            break;
        case CHAN2_FINISH:
            ndd_print(NDD_DEBUG, "CHAN2_FINISH\n");
            break;
        default:
            ndd_print(NDD_DEBUG, "ERROR!!!!\n");
            break;
    }
    ndd_print(NDD_DEBUG, "chipsel = %d\n",ops->bits.chipsel);
    ndd_print(NDD_DEBUG, "resource = %x\n",ops->bits.resource);
}

void dump_data(struct msgdata_data *data)
{
    ndd_print(NDD_DEBUG, "data->offset = %d\n",data->offset);
    ndd_print(NDD_DEBUG, "data->bytes = %d\n",data->bytes);
    ndd_print(NDD_DEBUG, "data->pdata = %x\n",data->pdata);
    ndd_print(NDD_DEBUG, "/****************************/\n");
}
void dump_cmd(struct msgdata_cmd *cmd)
{
    ndd_print(NDD_DEBUG, "cmd->command = %x\n",cmd->command);
    ndd_print(NDD_DEBUG, "cmd->cmddelay = %d\n",cmd->cmddelay);
    ndd_print(NDD_DEBUG, "cmd->addrdelay = %d\n",cmd->addrdelay);
    ndd_print(NDD_DEBUG, "cmd->offset = %d\n",cmd->offset);
    ndd_print(NDD_DEBUG, "cmd->pageid = %d\n",cmd->pageid);
    ndd_print(NDD_DEBUG, "/****************************/\n");
}
void dump_prepare(struct msgdata_prepare *prepare)
{
    ndd_print(NDD_DEBUG, "prepare->unit = %d\n",prepare->unit);
    ndd_print(NDD_DEBUG, "prepare->eccbit = %d\n",prepare->eccbit);
    ndd_print(NDD_DEBUG, "prepare->totaltasknum = %d\n",prepare->totaltasknum);
    ndd_print(NDD_DEBUG, "prepare->retnum = %d\n",prepare->retnum);
    ndd_print(NDD_DEBUG, "/****************************/\n");
}
void dump_ret(struct msgdata_ret *ret)
{
    ndd_print(NDD_DEBUG, "ret->bytes = %d\n",ret->bytes);
    ndd_print(NDD_DEBUG, "ret->retaddr = %x\n",ret->retaddr);
    ndd_print(NDD_DEBUG, "/****************************/\n");
}
void dump_parity(struct msgdata_parity *parity)
{
    ndd_print(NDD_DEBUG, "parity->offset = %d\n",parity->offset);
    ndd_print(NDD_DEBUG, "parity->bytes = %d\n",parity->bytes);
    ndd_print(NDD_DEBUG, "parity->parityaddr = %x\n",parity->parityaddr);
    ndd_print(NDD_DEBUG, "/****************************/\n");
}
void dump_badblock(struct msgdata_badblock *badblock)
{
    ndd_print(NDD_DEBUG, "badblock->planes = %d\n",badblock->planes);
    ndd_print(NDD_DEBUG, "badblock->blockid = %d\n",badblock->blockid);
    ndd_print(NDD_DEBUG, "/****************************/\n");
}
void dump_taskmsg(struct task_msg *msg, int num)
{
    union taskmsghead *ops;
    union taskmsgdata *msgdata;
    int i;
    ndd_print(NDD_DEBUG, "**** the total count of taskmsg is %d ****\n",num);
    for(i = 0; i < num; i++){
        ops = &((msg + i)->ops);
        msgdata = &((msg + i)->msgdata);
        dump_taskmsg_ops(ops);
        switch(ops->bits.type){
            case MSG_MCU_INIT:
            case MSG_MCU_PREPARE:
                    dump_prepare(&(msgdata->prepare));
                    break;
            case MSG_MCU_CMD:
                    dump_cmd(&(msgdata->cmd));
                    break;
            case MSG_MCU_BADBLOCK:
                    dump_badblock(&(msgdata->badblock));
                    break;
            case MSG_MCU_DATA:
                    dump_data(&(msgdata->data));
                    break;
            case MSG_MCU_RET:
                    dump_ret(&(msgdata->ret));
                    break;
            case MSG_MCU_PARITY:
                    dump_parity(&(msgdata->parity));
                    break;
            default:
                ndd_print(NDD_DEBUG, "ERROR!!!!\n");
                break;
        }
    }
}


