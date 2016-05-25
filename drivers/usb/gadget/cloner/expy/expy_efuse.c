#include <asm-generic/errno.h>
#include "../extend_policy_manager.h"
#include <efuse.h>

struct efuse_op {
	int gpio;
	int offset;
	int ops;  /*ops = 1 write*/
	int length;
	long long data;
};
struct efuse_priv {
	char *transfer_buf;
	int trans_len;
};


int expy_efuse_write(void *buf, int length, void *data)
{
	printf("%s\n", __func__);
	struct efuse_op *op = (struct efuse_op *)buf;
	struct efuse_priv *priv = (struct efuse_priv *)data;

	int r = 0;
	int len = op->length;
	priv->transfer_buf = malloc(sizeof(char)*len);
	memset(priv->transfer_buf, 0, len);
	sprintf(priv->transfer_buf,"%lld",op->data);


	printf("gpio = %d, data = %lld, length = %d, offset = %d, ops = %s\n",
			op->gpio,op->data,op->length, op->offset,op->ops?"write":"read");

	if ((r = efuse_init(op->gpio)) < 0) {
		printf("efuse init error\n");
		return r;
	}

	if (op->ops) {
		if (!!(r = efuse_write(priv->transfer_buf, len, op->offset))) {
			printf("expy_efuse write error\n");
			return r;
		}
	} else {
		if ((r = efuse_read(priv->transfer_buf, len, op->offset)) < 0) {
			printf("expy_efuse read error\n");
			return r;
		}
	}

	priv->trans_len = r * 4;
	return 0;
}

int expy_efuse_read(void *buf, int length, void *data)
{
	printf("%s\n", __func__);
	struct efuse_priv *priv = (struct efuse_priv *)data;
	memcpy(buf, (char *)priv->transfer_buf, priv->trans_len);
	return length;
}

int expy_efuse_init(void)
{
	struct extend_policy *expy = malloc(sizeof(struct extend_policy));
	if (!expy)
		return -ENOMEM;
	expy->magic = 0x30545847;
	expy->write = expy_efuse_write;
	expy->read = expy_efuse_read;
	expy->data = (struct efuse_priv *)malloc(sizeof(struct efuse_priv));
	printf("expy efuse register\n");
	return epmg_register(expy);
}

CLONER_SUB_MOUDLE_INIT(expy_efuse_init);
