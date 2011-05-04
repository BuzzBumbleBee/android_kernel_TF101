/*
 * (C) Copyright 2010 ASUS International Ltd.
 * All Rights Reserved
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/socket.h>
#include <linux/version.h>
#include <linux/kthread.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include <net/sock.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>

#ifdef CONFIG_ANDROID_PARANOID_NETWORK
#include <linux/android_aid.h>
#endif

#define RADIO_VERSION KERNEL_VERSION(0, 0, 1)

static int radio_nr = -1;

#define FM_RESET			0x00
#define FM_RECEIVER_INIT	0x01
#define FM_SET_MODE			0x02

#define FM_SET_CHANNEL		0x03
#define FM_GET_CHANNEL		0x04

#define FM_SET_SEARCH_MODE	0x09

#define FM_SET_BAND			0x13

#define FM_SET_MUTE			0x15
#define FM_GET_MUTE			0x16

#define FM_SET_AUDIO_PATH	0x1A
#define FM_GET_AUDIO_PATH	0x1B

#define FM_SET_INT_MASK		0x2E
#define FM_GET_INT_MASK		0x2F

#define FM_SET_SAMPLE_RATE	0x3F

#define FM_SET_VOLUME		0x65
#define FM_GET_VOLUME		0x66


#define FM_EVENT_RDS_DATA	0x81
#define FM_EVENT_CUR_RSSI	0x82
#define FM_EVENT_CUR_MOST	0x83
#define FM_EVENT_AUD_PASI	0x84
#define FM_EVENT_RDS_LOST	0x85
#define FM_EVENT_LOW_RSSI	0x86
#define FM_EVENT_LOW_CMI	0x87
#define FM_EVENT_HCI_GENE	0xA0


#define FM_FREQ_LOW			76000
#define FM_FREQ_HIGT		108000

#define BUF_SIZE_RECV		256
#define BUF_SIZE_RDS		256
#define DEPTH_RDS			5
#define DEPTH_EVENTS		5

//Asus Broadcom Spec
#define I2S_FM_RDS_SYSTEM		0x00
#define I2S_FM_AUDIO_CTRL0		0x05
#define I2S_FM_FREQ0			0x0A
#define I2S_FM_FREQ1			0x0B
#define I2S_FM_SEARCH_TUNE_MODE	0x09


#define ASUS4329_CID_BASE	(V4L2_CID_LASTP1+1)


struct cmdrequest {
	uint8_t		idx;
	uint8_t		len;
	uint32_t	val;
};

struct asus4329_device {
	struct v4l2_device	v4l2_dev;
	struct video_device	vdev;
	struct mutex		lock;
	struct socket		*sock;

	struct task_struct	*thread;
	int					stop_thread;

	struct completion	*cmd_done;
	struct cmdrequest	request;
	unsigned char		recvbuf[BUF_SIZE_RECV];

	wait_queue_head_t	read_queue;
	unsigned char		rdsin;
	unsigned char		rdsout;
	unsigned char		rdsbuf[DEPTH_RDS][BUF_SIZE_RDS];

	unsigned char		ein;
	unsigned char		eout;
	unsigned long		ebuf[DEPTH_EVENTS];
};

static struct asus4329_device asus4329_dev;


static int asus4329_hci_open_device(struct asus4329_device *dev)
{
	struct sockaddr address;
	struct hci_filter flt;
	int err = 0;

	err = sock_create_kern(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI, &dev->sock);
	if (err < 0) {
		printk(KERN_ERR "[fm] asus4329_hci_open_device: failed to sock_create_kern\n");
		return err;
	}

	memset(&address, 0, sizeof(address));
	address.sa_family = AF_BLUETOOTH;
	err = kernel_bind(dev->sock, &address, sizeof(address));
	if (err < 0) {
		printk(KERN_ERR "[fm] asus4329_hci_open_device: failed to kernel_bind\n");
		goto error;
	}

	memset(&flt, 0, sizeof(flt));
	flt.type_mask = (1 << HCI_EVENT_PKT);
	flt.event_mask[0] = 0xFFFFFFFF;
	flt.event_mask[1] = 0xFFFFFFFF;

	err = kernel_setsockopt(dev->sock, SOL_HCI, HCI_FILTER, (char *)&flt, sizeof(flt));
	if (err < 0) {
		printk(KERN_ERR "[fm] asus4329_hci_open_device: failed to kernel_setsockopt\n");
		goto error;
	}

	return err;

error:
	sock_release(dev->sock);
	dev->sock = NULL;

	return -1;
}

static int asus4329_hci_sendcmd(struct asus4329_device *dev, uint8_t idx, uint32_t val)
{
	struct socket *sock = dev->sock;
	uint8_t type = HCI_COMMAND_PKT;
	struct kvec vec[3];
	struct hci_command_hdr hc;
	struct msghdr msg;
	char cmdbuf[8];
	uint8_t cmdlen;
	int err = 0;
	//Kyle len 4->2
	const int len = 2;

	dev->request.idx = idx;
	dev->request.val = val;
	dev->request.len = len;

	cmdbuf[0] = idx;
	memcpy(&cmdbuf[1], &val, sizeof(val));
	cmdlen = len + 1;

	hc.opcode = 0xFC15;
	hc.plen = cmdlen;

	vec[0].iov_base = &type;
	vec[0].iov_len  = 1;

	vec[1].iov_base = &hc;
	vec[1].iov_len  = HCI_COMMAND_HDR_SIZE;

	vec[2].iov_base = cmdbuf;
	vec[2].iov_len  = cmdlen;

	memset(&msg, 0, sizeof(msg));

	err = kernel_sendmsg(sock, &msg, vec, 3, cmdlen+4);
	if (err < 0)
		printk(KERN_ERR "[fm] asus4329_hci_sendcmd: failed to kernel_sendmsg, return: %d\n", err);

	return err;
}

static int asus4329_hci_sendcmd_sync(struct asus4329_device *dev, uint8_t idx, uint32_t val)
{
	DECLARE_COMPLETION_ONSTACK(complete);

	mutex_lock(&dev->lock);
	if (dev->sock) {
		dev->cmd_done = &complete;
		mutex_unlock(&dev->lock);
	} else {
		mutex_unlock(&dev->lock);
		return -1;
	}

	asus4329_hci_sendcmd(dev, idx, val);

	wait_for_completion(&complete);
	return 0;
}

static int asus4329_hci_recvmsg(struct socket *sock, char *pbuf, uint32_t len)
{
	struct msghdr msg;
	struct kvec rvec;
	int ret = 0;

	memset(&msg, 0, sizeof(msg));

	rvec.iov_base = pbuf;
	rvec.iov_len = len;

	ret = kernel_recvmsg(sock, &msg, &rvec, 1, len, 0);
	if (ret < 0)
		printk(KERN_ERR "[fm] asus4329_hci_recvmsg: failed to kernel_recvmsg, return: %d\n", ret);

	return ret;
}


static int asus4329_hci_parse_response(struct asus4329_device *dev, char *buf, int len)
{
	int rlen;

        /**
	if (len < 9) {
		printk(KERN_ERR "[fm] hci_parse_cmdevent: buf len incorrect:%d\n", len);
		return -1;
	}
        **/

	/**
	if (buf[7] != dev->request.idx) {
		printk(KERN_ERR "[fm] hci_parse_cmdevent: command not match:%x:%x\n", buf[7], dev->request.idx);
		return -1;
	}
	**/

	rlen = buf[2];
        /**
	if ((rlen < 6) || (rlen > 10)) {
		printk(KERN_ERR "[fm] hci_parse_cmdevent: data len incorrect:%d\n", rlen);
		return -1;
	}
         **/

	rlen -= 6;

	dev->request.len = rlen;
	dev->request.val = 0;

	if (rlen) {
		memcpy(&dev->request.val, &buf[9], rlen);
	}

	if (dev->cmd_done){
		struct completion	*done;
		done = dev->cmd_done;
		dev->cmd_done = NULL;
		complete(done);
	}

	return 0;
}

static int asus4329_hci_parse_event(struct asus4329_device *dev, char *buf, int len)
{
	int size;
	char type;

	if (len < 4)
		return -1;

	type = buf[3];
	size = buf[2];

	if (size > len - 3)
		return -1;

	switch (type) {
		case FM_EVENT_RDS_DATA:
			memcpy(&dev->rdsbuf[dev->rdsin][0], &buf[2], size+1);
			if (dev->rdsin+1 == dev->rdsout)
				printk(KERN_WARNING "[fm] RDS buffer overflow\n");
			else
				dev->rdsin++;
			dev->rdsin = dev->rdsin % DEPTH_RDS;
			break;

		case FM_EVENT_CUR_RSSI:
		case FM_EVENT_CUR_MOST:
		case FM_EVENT_AUD_PASI:
		case FM_EVENT_RDS_LOST:
		case FM_EVENT_LOW_RSSI:
		case FM_EVENT_LOW_CMI:
		case FM_EVENT_HCI_GENE:
			dev->ebuf[dev->ein] = 0;
			memcpy(&dev->ebuf[dev->ein], &buf[3], size);
			if (dev->ein+1 == dev->eout)
				printk(KERN_WARNING "[fm] Event buffer overflow\n");
			else
				dev->ein++;
			dev->ein = dev->ein % DEPTH_EVENTS;
			break;
	}

	if ((dev->rdsin != dev->rdsout) || (dev->ein != dev->eout))
		wake_up_interruptible(&dev->read_queue);

	return 0;
}

static int asus4329_vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *v)
{
	strlcpy(v->driver, "radio-asus", sizeof(v->driver));
	strlcpy(v->card, "asus radio", sizeof(v->card));
	strlcpy(v->bus_info, "HCI", sizeof(v->bus_info));
	v->version = RADIO_VERSION;
	v->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO;
	return 0;
}

static int asus4329_vidioc_g_tuner(struct file *file, void *priv, struct v4l2_tuner *v)
{
	if (v->index > 0)
		return -EINVAL;

	strlcpy(v->name, "FM", sizeof(v->name));
	v->type = V4L2_TUNER_RADIO;
	v->rangelow = FM_FREQ_LOW;
	v->rangehigh = FM_FREQ_HIGT;
	v->rxsubchans = V4L2_TUNER_SUB_MONO;
	v->capability = V4L2_TUNER_CAP_LOW;
	v->audmode = V4L2_TUNER_MODE_MONO;
	v->signal = 0xFFFF;

	return 0;
}

static int asus4329_vidioc_s_tuner(struct file *file, void *priv, struct v4l2_tuner *v)
{
	return v->index ? -EINVAL : 0;
}

static int asus4329_vidioc_g_frequency(struct file *file, void *priv, struct v4l2_frequency *f)
{
	struct asus4329_device *dev = &asus4329_dev;
	//int retval;
	int retfreq0,retfreq1;

	f->type = V4L2_TUNER_RADIO;

	asus4329_hci_sendcmd_sync(dev, I2S_FM_FREQ0, 0x0101);
	retfreq0 = dev->request.val;
	asus4329_hci_sendcmd_sync(dev, I2S_FM_FREQ1, 0x0101);
	retfreq1 = dev->request.val;

	f->frequency = retfreq0 + retfreq1*256 + 64000;

	printk(KERN_ERR "[fm] freq is %x\n",f->frequency);

	/*
	if (retval >= 0) {
		f->frequency = dev->request.val;
	}
	*/

//	return retval;
	return 0;
}

static int asus4329_vidioc_s_frequency(struct file *file, void *priv,	struct v4l2_frequency *f)
{
	struct asus4329_device *dev = &asus4329_dev;

	printk(KERN_ERR "[fm] asus4329 set_freq start!\n");
	printk(KERN_ERR "[fm] freq is %x\n",f->frequency);

	if (f->frequency < FM_FREQ_LOW || f->frequency > FM_FREQ_HIGT) {
		return -EINVAL;
	}

	//test
	printk(KERN_ERR "[fm] freq1 is %x\n",((f->frequency - 64000)%256) * 256);
	printk(KERN_ERR "[fm] freq2 is %x\n",f->frequency - 64000 -  ((f->frequency-64000)%256));

                  //Kyle ++
	asus4329_hci_sendcmd_sync(dev, I2S_FM_FREQ0, (((f->frequency - 64000)%256) * 256) );
	asus4329_hci_sendcmd_sync(dev, I2S_FM_FREQ1, (f->frequency - 64000 -  ((f->frequency-64000)%256)) );
//                   asus4329_hci_sendcmd_sync(dev, I2S_FM_SEARCH_TUNE_MODE, 0x0100);
	

	printk(KERN_ERR "[fm] asus4329 set_freq done!\n");

//	return asus4329_hci_sendcmd_sync(dev, FM_SET_CHANNEL, f->frequency);
	return 0;
}


static int asus4329_vidioc_queryctrl(struct file *file, void *priv,	struct v4l2_queryctrl *qc)
{
	int i;
	struct v4l2_queryctrl asus4329_qctrl[] = {
		{
			.id = V4L2_CID_AUDIO_MUTE,
			.name = "Mute",
			.minimum = 0,
			.maximum = 1,
			.default_value = 1,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
		},
		{
			.id = V4L2_CID_AUDIO_VOLUME,
			.name = "Volume",
			.minimum = 0,
			.maximum = 2048,
			.step = 1,
			.default_value = 0x4F,
			.type = V4L2_CTRL_TYPE_INTEGER,
		},
	};

	for (i = 0; i < ARRAY_SIZE(asus4329_qctrl); ++i) {
		if (qc->id && qc->id == asus4329_qctrl[i].id) {
			memcpy(qc, &(asus4329_qctrl[i]), sizeof(*qc));
			return 0;
		}
	}
	return -EINVAL;
}

static int asus4329_vidioc_g_ctrl(struct file *file, void *priv,
					struct v4l2_control *ctrl)
{
	struct asus4329_device *dev = &asus4329_dev;
	int retval;

	if (ctrl->id >= ASUS4329_CID_BASE && ctrl->id <= (ASUS4329_CID_BASE+0xFF)) {
		retval = asus4329_hci_sendcmd_sync(dev, ctrl->id - ASUS4329_CID_BASE, 0);
		ctrl->value = dev->request.val;
		return retval;
	}

	switch (ctrl->id) {
		case V4L2_CID_AUDIO_MUTE:
			retval = asus4329_hci_sendcmd_sync(dev, FM_GET_MUTE, 0);
			ctrl->value = dev->request.val;
			break;

		case V4L2_CID_AUDIO_VOLUME:
			retval = asus4329_hci_sendcmd_sync(dev, FM_GET_VOLUME, 0);
			ctrl->value = dev->request.val;
			break;

		default:
			return -EINVAL;
	}

	return retval;
}

static int asus4329_vidioc_s_ctrl (struct file *file, void *priv,
					struct v4l2_control *ctrl)
{
	struct asus4329_device *dev = &asus4329_dev;
	int retval;

    if (ctrl->id >= ASUS4329_CID_BASE && ctrl->id <= (ASUS4329_CID_BASE+0xFF)) {
		retval = asus4329_hci_sendcmd_sync(dev, ctrl->id - ASUS4329_CID_BASE, ctrl->value);
		return retval;
	}

	switch (ctrl->id) {
		case V4L2_CID_AUDIO_MUTE:
			retval = asus4329_hci_sendcmd_sync(dev, FM_SET_MUTE, ctrl->value ? 0x06 : 0);
			break;

		case V4L2_CID_AUDIO_VOLUME:
			retval = asus4329_hci_sendcmd_sync(dev, FM_SET_VOLUME, ctrl->value & 0x7FF);
			break;

		default:
			return -EINVAL;
	}

	return retval;
}

static int asus4329_vidioc_g_input(struct file *filp, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int asus4329_vidioc_s_input(struct file *filp, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static int asus4329_vidioc_g_audio(struct file *file, void *priv,
					struct v4l2_audio *a)
{
	a->index = 0;
	strlcpy(a->name, "Radio", sizeof(a->name));
	a->capability = V4L2_AUDCAP_STEREO;
	return 0;
}

static int asus4329_vidioc_s_audio(struct file *file, void *priv,
					struct v4l2_audio *a)
{
	return a->index ? -EINVAL : 0;
}

static int asus4329_thread(void *d)
{
	struct asus4329_device *dev = d;
	int len;

	while (!dev->stop_thread) {
		len = asus4329_hci_recvmsg(dev->sock, dev->recvbuf, BUF_SIZE_RECV);

		if (len < 0)
			break;

		if (len < 2)
			continue;

		switch (dev->recvbuf[1]) {
			case 0x0E:
				asus4329_hci_parse_response(dev, dev->recvbuf, len);
				break;

			case 0xFF:
				asus4329_hci_parse_event(dev, dev->recvbuf, len);
				break;
		}
	}

	mutex_lock(&dev->lock);
	if (dev->sock) {
		sock_release(dev->sock);
		dev->sock = NULL;
	}
	if (dev->cmd_done) {
		complete(dev->cmd_done);
		dev->cmd_done = NULL;
	}
	mutex_unlock(&dev->lock);

	printk(KERN_INFO "[fm] asus4329_thread exit......\n");
	dev->thread = NULL;
	return 0;
}

#ifdef CONFIG_ANDROID_PARANOID_NETWORK
static int asus4329_setcaps(gid_t grp)
{
	const struct cred *cred = current_cred();
	struct group_info *giold;
	struct group_info *ginew;
	unsigned int count;
	int i;
	int retval;

	if (in_egroup_p(grp))
		return 0;

	giold = cred->group_info;
	count = giold->ngroups;

	ginew = groups_alloc(count + 1);

	for (i = 0; i < count; i++)
		GROUP_AT(ginew, i) = GROUP_AT(giold, i);
	GROUP_AT(ginew, count) = grp;

	retval = set_current_groups(ginew);
	put_group_info(ginew);

	return retval;
}
#endif

static int asus4329_fops_open(struct file *file)
{
	struct asus4329_device *dev = &asus4329_dev;
	int err = 0;

#ifdef CONFIG_ANDROID_PARANOID_NETWORK
	asus4329_setcaps(AID_NET_BT_ADMIN);
	asus4329_setcaps(AID_NET_RAW);
#endif

	dev->cmd_done = NULL;
	dev->rdsin = 0;
	dev->rdsout = 0;
	dev->ein = 0;
	dev->eout = 0;

	init_waitqueue_head(&dev->read_queue);

	printk(KERN_ERR "[fm] asus4329 open!\n");

	err = asus4329_hci_open_device(dev);
	if (err < 0) {
		printk(KERN_ERR "[fm] failed to open hci device!\n");
		return err;
	}

	printk(KERN_ERR "[fm] asus4329 -  asus4329_hci_open_device done!\n");

	dev->stop_thread = 0;
	dev->thread = kthread_run(asus4329_thread, dev, "asusfm");
	if (IS_ERR(dev->thread)) {
		printk(KERN_ERR "[fm] asus4329_fops_open: fail to kthread_run\n");
		return PTR_ERR(dev->thread);
	}

	printk(KERN_ERR "[fm] asus4329 - kthread_run done!\n");
/*
	asus4329_hci_sendcmd_sync(dev, FM_RESET, 0);
	asus4329_hci_sendcmd_sync(dev, FM_SET_MODE, 0x00);
	asus4329_hci_sendcmd_sync(dev, FM_RECEIVER_INIT, 0x018CBA80);
	asus4329_hci_sendcmd_sync(dev, FM_SET_AUDIO_PATH, 0x000000);
	asus4329_hci_sendcmd_sync(dev, FM_SET_SAMPLE_RATE, 0x0003);
	asus4329_hci_sendcmd_sync(dev, FM_SET_SEARCH_MODE, 0x01);
	asus4329_hci_sendcmd_sync(dev, FM_SET_BAND, 0x03);
	asus4329_hci_sendcmd_sync(dev, FM_SET_VOLUME, 0x4F);
	asus4329_hci_sendcmd_sync(dev, FM_SET_CHANNEL, 91400);
	asus4329_hci_sendcmd_sync(dev, FM_SET_MODE, 0x03);
	asus4329_hci_sendcmd_sync(dev, FM_SET_INT_MASK, 0x07);
*/
	asus4329_hci_sendcmd_sync(dev, I2S_FM_RDS_SYSTEM, 0x0100);
	asus4329_hci_sendcmd_sync(dev, I2S_FM_AUDIO_CTRL0, 0x1C00);
	asus4329_hci_sendcmd_sync(dev, I2S_FM_FREQ0, 0x7400);
	asus4329_hci_sendcmd_sync(dev, I2S_FM_FREQ1, 0x8B00);
                   asus4329_hci_sendcmd_sync(dev, I2S_FM_SEARCH_TUNE_MODE, 0x0100);

	printk(KERN_ERR "[fm] asus4329 hci_sendcmd done!\n");
	return 0;
}

static int asus4329_fops_release(struct file *file)
{
	struct asus4329_device *dev = &asus4329_dev;

	if (dev->thread) {
		asus4329_hci_sendcmd_sync(dev, FM_SET_INT_MASK, 0x00);
		asus4329_hci_sendcmd_sync(dev, FM_SET_VOLUME, 0x0000);
		dev->stop_thread = 1;
		asus4329_hci_sendcmd_sync(dev, FM_SET_MODE, 0x00);
		while (dev->thread)
			msleep(10);
	}

	if (dev->sock) {
		sock_release(dev->sock);
		dev->sock = NULL;
	}

	printk(KERN_INFO "[fm] asus4329_fops_release exit......\n");
	return 0;
}

static ssize_t asus4329_fops_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct asus4329_device *dev = &asus4329_dev;
	int retval = 0;
	unsigned char size = 0;

	while ((dev->rdsin == dev->rdsout) && (dev->ein == dev->eout)) {
		if (file->f_flags & O_NONBLOCK) {
			retval = -EWOULDBLOCK;
			goto done;
		}
		if (wait_event_interruptible(dev->read_queue,
				((dev->rdsin != dev->rdsout) || (dev->ein != dev->eout))) < 0) {
			retval = -EINTR;
			goto done;
		}
	}

	if (dev->rdsin != dev->rdsout) {
		size = dev->rdsbuf[dev->rdsout][0];
		if (count < size) {
			retval = -EINTR;
			goto done;
		}
		retval = copy_to_user(buf, &dev->rdsbuf[dev->rdsout][1], size);
		dev->rdsout++;
		dev->rdsout = dev->rdsout % DEPTH_RDS;
	} else if (dev->ein != dev->eout) {
		size = 4;
		if (count < size) {
			retval = -EINTR;
			goto done;
		}
		retval = copy_to_user(buf, &dev->ebuf[dev->eout], size);
		dev->eout++;
		dev->eout = dev->eout % DEPTH_EVENTS;
	}

done:
	return size;
}

static unsigned int asus4329_fops_poll(struct file *file,
		struct poll_table_struct *pts)
{
	struct asus4329_device *dev = &asus4329_dev;
	int retval = 0;

	poll_wait(file, &dev->read_queue, pts);

	if ((dev->rdsin != dev->rdsout) || (dev->ein != dev->eout))
		retval = POLLIN | POLLRDNORM;

	return retval;
}

static const struct v4l2_file_operations asus4329_fops = {
	.owner		= THIS_MODULE,
	.open		= asus4329_fops_open,
	.release	= asus4329_fops_release,
	.ioctl		= video_ioctl2,
	.read		= asus4329_fops_read,
	.poll		= asus4329_fops_poll,
};

static const struct v4l2_ioctl_ops asus4329_ioctl_ops = {
	.vidioc_querycap    = asus4329_vidioc_querycap,
	.vidioc_g_tuner     = asus4329_vidioc_g_tuner,
	.vidioc_s_tuner     = asus4329_vidioc_s_tuner,
	.vidioc_g_audio     = asus4329_vidioc_g_audio,
	.vidioc_s_audio     = asus4329_vidioc_s_audio,
	.vidioc_g_input     = asus4329_vidioc_g_input,
	.vidioc_s_input     = asus4329_vidioc_s_input,
	.vidioc_g_frequency = asus4329_vidioc_g_frequency,
	.vidioc_s_frequency = asus4329_vidioc_s_frequency,
	.vidioc_queryctrl   = asus4329_vidioc_queryctrl,
	.vidioc_g_ctrl      = asus4329_vidioc_g_ctrl,
	.vidioc_s_ctrl      = asus4329_vidioc_s_ctrl,
};

static int __init asus4329_init(void)
{
	struct asus4329_device *mdev = &asus4329_dev;
	struct v4l2_device *v4l2_dev = &mdev->v4l2_dev;
	int res;

	strlcpy(v4l2_dev->name, "asus4329", sizeof(v4l2_dev->name));
	mutex_init(&mdev->lock);

	printk(KERN_ERR "[fm] asus4329 init\n");

	res = v4l2_device_register(NULL, v4l2_dev);
	if (res < 0) {
		printk(KERN_ERR "[fm] Could not register v4l2_device\n");
		return res;
	}

	strlcpy(mdev->vdev.name, v4l2_dev->name, sizeof(mdev->vdev.name));
	mdev->vdev.v4l2_dev = v4l2_dev;
	mdev->vdev.fops = &asus4329_fops;
	mdev->vdev.ioctl_ops = &asus4329_ioctl_ops;
	mdev->vdev.release = video_device_release_empty;

	if (video_register_device(&mdev->vdev, VFL_TYPE_RADIO, radio_nr) < 0) {
		v4l2_device_unregister(v4l2_dev);
		printk(KERN_ERR "[fm] asus4329_init: fail to video_register_device\n");
		return -EINVAL;
	}

	v4l2_info(v4l2_dev, "Asus FM Radio card driver v1.0.\n");
	return 0;
}

static void __exit asus4329_exit(void)
{
	struct asus4329_device *mdev = &asus4329_dev;

	video_unregister_device(&mdev->vdev);
	v4l2_device_unregister(&mdev->v4l2_dev);
}

module_init(asus4329_init);
module_exit(asus4329_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Asus FM radio driver");
