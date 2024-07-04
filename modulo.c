#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/hid.h>
#include <linux/input-event-codes.h>


#define XPAD_PKT_LEN 64
#define XTYPE_XBOXONE 3

#define XPAD_OUT_CMD_IDX	0

#define XPAD_NUM_OUT_PACKETS 1

struct xpad_output_packet {
	u8 data[XPAD_PKT_LEN];
	u8 len;
	bool pending;
};

int array [200];
int n = 0;


struct usb_xpad {
    struct input_dev *dev;
	struct usb_device *udev;
	struct usb_interface *intf;

	bool pad_present;
	bool input_created;

	struct urb *irq_in;
	unsigned char *idata;
	dma_addr_t idata_dma;

	struct urb *irq_out;
	struct usb_anchor irq_out_anchor;
	bool irq_out_active;
	u8 odata_serial;
	unsigned char *odata;
	dma_addr_t odata_dma;
	spinlock_t odata_lock;

	struct xpad_output_packet out_packets[XPAD_NUM_OUT_PACKETS];
	int last_out_packet;
	int init_seq;

#if defined(CONFIG_JOYSTICK_XPAD_LEDS)
	struct xpad_led *led;
#endif

	char phys[64];

	int mapping;
	int xtype;
	int packet_type;
	int pad_nr;
	int quirks;
	char name[64];
	struct work_struct work;
	struct delayed_work poweroff_work;
	time64_t mode_btn_down_ts;
	struct urb *ghl_urb;
	struct timer_list ghl_poke_timer;
};


static void process_packet(struct usb_xpad *xpad, u16 cmd, unsigned char *data){

	// ----------------- LETRAS -----------------

	if ((int)data[3] & 0b00010000){		// A pressed
		input_report_key(xpad->dev, KEY_LEFTCTRL , 1);
		input_report_key(xpad->dev, KEY_C , 1);
	} else {
		input_report_key(xpad->dev, KEY_LEFTCTRL , 0);
		input_report_key(xpad->dev, KEY_C , 0);
	}

	if ((int)data[3] & 0b00100000){		// B pressed
		input_report_key(xpad->dev, KEY_LEFTCTRL , 1);
		input_report_key(xpad->dev, KEY_V , 1);
	} else {
		input_report_key(xpad->dev, KEY_LEFTCTRL , 0);
		input_report_key(xpad->dev, KEY_V , 0);
	}

	if ((int)data[3] & 0b01000000){		// X pressed
		input_report_key(xpad->dev, KEY_LEFTCTRL , 1);
		input_report_key(xpad->dev, KEY_Z , 1);
	} else {
		input_report_key(xpad->dev, KEY_LEFTCTRL , 0);
		input_report_key(xpad->dev, KEY_Z , 0);
	}

	if ((int)data[3] & 0b10000000){		// Y pressed
		input_report_key(xpad->dev, KEY_SPACE , 1);
	} else {
		input_report_key(xpad->dev, KEY_SPACE , 0);
	}

	// ----------------- DIRECIONAIS -----------------

	if ((int)data[2] & 0b0001){		// Direcional -> Up
		input_report_key(xpad->dev, KEY_UP , 1);
	} else {
		input_report_key(xpad->dev, KEY_UP , 0);
	}

	if ((int)data[2] & 0b0010){		// Direcional -> Down
		input_report_key(xpad->dev, KEY_DOWN , 1);
	} else {
		input_report_key(xpad->dev, KEY_DOWN , 0);
	}

	if ((int)data[2] & 0b0100){		// Direcional -> Esq
		input_report_key(xpad->dev, KEY_LEFT , 1);
	} else {
		input_report_key(xpad->dev, KEY_LEFT , 0);
	}

	if ((int)data[2] & 0b1000){		// Direcional -> Dir
		input_report_key(xpad->dev, KEY_RIGHT , 1);
	} else {
		input_report_key(xpad->dev, KEY_RIGHT , 0);
	}

	// ----------------- LB e RB -----------------

	if ((int)data[3] & 0b01){		// LB
		input_report_key(xpad->dev, KEY_LEFTSHIFT , 1);
	} else {
		input_report_key(xpad->dev, KEY_LEFTSHIFT , 0);
	}

	if ((int)data[3] & 0b10){		// RB
		input_report_key(xpad->dev, KEY_BACKSPACE , 1);
	} else {
		input_report_key(xpad->dev, KEY_BACKSPACE , 0);
	}

	// ----------------- LT e RT -----------------

	if ((int)data[4] & 255){		// LT
		input_report_key(xpad->dev, KEY_LEFTALT , 1);
		input_report_key(xpad->dev, KEY_TAB , 1);
	} else {
		input_report_key(xpad->dev, KEY_LEFTALT , 0);
		input_report_key(xpad->dev, KEY_TAB , 0);
	}

	if ((int)data[5] & 255){		// RT
		input_report_key(xpad->dev, KEY_ENTER , 1);
	} else {
		input_report_key(xpad->dev, KEY_ENTER , 0);
	}

	// ----------------- Analógicos -----------------
	/*
	 * 		<: 128    o: 255		id left: 7			id right: 11
	 * 		>: 127    o: 0			id left: 7			id right: 11
	 * 		^: 127    o: 0			id left: 9			id right: 13
	 * 		v: 128    o: 255		id left: 9			id right: 13
	 * */

	if ((int)data[7] == 128  &&  (int)data[11] == 128)	// <<
		input_report_key(xpad->dev, KEY_A , 1);
	else
		input_report_key(xpad->dev, KEY_A , 0);

	if ((int)data[7] == 128  &&  (int)data[11] == 127)	// <>
		input_report_key(xpad->dev, KEY_I , 1);
	else
		input_report_key(xpad->dev, KEY_I , 0);

	if ((int)data[7] == 128  &&  (int)data[13] == 127)	// <^
		input_report_key(xpad->dev, KEY_E , 1);
	else
		input_report_key(xpad->dev, KEY_E , 0);

	if ((int)data[7] == 128  &&  (int)data[13] == 128)	// <v
		input_report_key(xpad->dev, KEY_O , 1);
	else
		input_report_key(xpad->dev, KEY_O , 0);



	if ((int)data[7] == 127  &&  (int)data[11] == 128)	// ><
		input_report_key(xpad->dev, KEY_L , 1);
	else
		input_report_key(xpad->dev, KEY_L , 0);

	if ((int)data[7] == 127  &&  (int)data[11] == 127)	// >>
		input_report_key(xpad->dev, KEY_N , 1);
	else
		input_report_key(xpad->dev, KEY_N , 0);

	if ((int)data[7] == 127  &&  (int)data[13] == 127)	// >^
		input_report_key(xpad->dev, KEY_S , 1);
	else
		input_report_key(xpad->dev, KEY_S , 0);

	if ((int)data[7] == 127  &&  (int)data[13] == 128)	// >v
		input_report_key(xpad->dev, KEY_K , 1);
	else
		input_report_key(xpad->dev, KEY_K , 0);



	if ((int)data[9] == 128  &&  (int)data[11] == 128)	// v<
		input_report_key(xpad->dev, KEY_P , 1);
	else
		input_report_key(xpad->dev, KEY_P , 0);

	if ((int)data[9] == 128  &&  (int)data[11] == 127)	// v>
		input_report_key(xpad->dev, KEY_C , 1);
	else
		input_report_key(xpad->dev, KEY_C , 0);

	if ((int)data[9] == 128  &&  (int)data[13] == 127)	// v^
		input_report_key(xpad->dev, KEY_R , 1);
	else
		input_report_key(xpad->dev, KEY_R , 0);

	/*if ((int)data[9] == 128  &&  (int)data[13] == 128)	// vv
		input_report_key(xpad->dev, KEY_O , 1);
	else
		input_report_key(xpad->dev, KEY_O , 0);*/



	if ((int)data[9] == 127  &&  (int)data[11] == 128)	// ^<
		input_report_key(xpad->dev, KEY_M , 1);
	else
		input_report_key(xpad->dev, KEY_M , 0);

	if ((int)data[9] == 127  &&  (int)data[11] == 127)	// ^>
		input_report_key(xpad->dev, KEY_T , 1);
	else
		input_report_key(xpad->dev, KEY_T , 0);

	if ((int)data[9] == 127  &&  (int)data[13] == 127)	// ^^
		input_report_key(xpad->dev, KEY_Y , 1);
	else
		input_report_key(xpad->dev, KEY_Y , 0);
/*
	if ((int)data[9] == 127  &&  (int)data[13] == 128)	// ^v
		input_report_key(xpad->dev, KEY_S , 1);
	else
		input_report_key(xpad->dev, KEY_S , 0);*/


/*
	if ((int)data[7])
		printk(KERN_INFO "analog: %d", (int)data[7]);

	if ((int)data[7] == 128)
		printk(KERN_INFO "<<<");
	else if ((int)data[7] > 128 && (int)data[7] < 200)
		printk(KERN_INFO "<<");
	else if ((int)data[7] > 200 && (int)data[7] < 255)
		printk(KERN_INFO "<");

	if ((int)data[7] < 80 && (int)data[7] > 0)
		printk(KERN_INFO ">");
	else if ((int)data[7] > 80 && (int)data[7] < 127)
		printk(KERN_INFO ">>");
	else if ((int)data[7] == 127)
		printk(KERN_INFO ">>>");

	if ((int)data[3] & 0b00010000){		// A pressed
		input_report_key(xpad->dev, KEY_UP , 1);
		input_report_key(xpad->dev, KEY_ENTER , 1);
	} else {
		input_report_key(xpad->dev, KEY_UP , 0);
		input_report_key(xpad->dev, KEY_ENTER , 0);
	}
*/

	input_sync(xpad->dev);
}


static void xpad_irq_in(struct urb *urb)
{
	struct usb_xpad *xpad = urb->context;
	struct device *dev = &xpad->intf->dev;

	int status;

	status = urb->status;

	switch (status) {
	case 0:
		//printk(KERN_INFO "sucessful");
		break;

	case -ECONNRESET:
		printk(KERN_INFO "econnreset");
	case -ENOENT:
		printk(KERN_INFO "enoent");
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(dev, "%s - urb shutting down with status: %d\n",
			__func__, status);
		return;
	default:
		dev_dbg(dev, "%s - nonzero urb status received: %d\n", __func__, status);
	}


	//print_hex_dump(KERN_DEBUG, "xpad-dbg: ", DUMP_PREFIX_OFFSET, 32, 1, xpad->idata, XPAD_PKT_LEN, 0);

	process_packet(xpad, 0, xpad->idata);

	//int error;
	usb_submit_urb(xpad->irq_in, GFP_KERNEL);
}


static bool xpad_prepare_next_out_packet(struct usb_xpad *xpad)
{
	struct xpad_output_packet *pkt, *packet = NULL;
	int i;


	for (i = 0; i < XPAD_NUM_OUT_PACKETS; i++) {
		if (++xpad->last_out_packet >= XPAD_NUM_OUT_PACKETS)
			xpad->last_out_packet = 0;

		pkt = &xpad->out_packets[xpad->last_out_packet];
		if (pkt->pending) {
			dev_dbg(&xpad->intf->dev,
				"%s - found pending output packet %d\n",
				__func__, xpad->last_out_packet);
			packet = pkt;
			break;
		}
	}

	if (packet) {
		memcpy(xpad->odata, packet->data, packet->len);
		xpad->irq_out->transfer_buffer_length = packet->len;
		packet->pending = false;
		return true;
	}

	return false;
}


static int xpad_try_sending_next_out_packet(struct usb_xpad *xpad)
{
	int error;

	if (!xpad->irq_out_active && xpad_prepare_next_out_packet(xpad)) {
		usb_anchor_urb(xpad->irq_out, &xpad->irq_out_anchor);
		error = usb_submit_urb(xpad->irq_out, GFP_ATOMIC);
		if (error) {
			dev_err(&xpad->intf->dev,
				"%s - usb_submit_urb failed with result %d\n",
				__func__, error);
			usb_unanchor_urb(xpad->irq_out);
			return -EIO;
		}

		xpad->irq_out_active = true;
	}

	return 0;
}

static int xpad_inquiry_pad_presence(struct usb_xpad *xpad)
{
	struct xpad_output_packet *packet =
			&xpad->out_packets[XPAD_OUT_CMD_IDX];
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&xpad->odata_lock, flags);

	packet->data[0] = 0x08;
	packet->data[1] = 0x00;
	packet->data[2] = 0x0F;
	packet->data[3] = 0xC0;
	packet->data[4] = 0x00;
	packet->data[5] = 0x00;
	packet->data[6] = 0x00;
	packet->data[7] = 0x00;
	packet->data[8] = 0x00;
	packet->data[9] = 0x00;
	packet->data[10] = 0x00;
	packet->data[11] = 0x00;
	packet->len = 12;
	packet->pending = true;

	/* Reset the sequence so we send out presence first */
	xpad->last_out_packet = -1;
	retval = xpad_try_sending_next_out_packet(xpad);

	spin_unlock_irqrestore(&xpad->odata_lock, flags);

	return retval;
}


static int xpad360w_start_input(struct usb_xpad *xpad)
{
	int error;

	printk(KERN_INFO "xpad360 foi chamada");

	error = usb_submit_urb(xpad->irq_in, GFP_KERNEL);

	printk(KERN_INFO "erro: %d", error);


	if (error)
		return -EIO;

	/*
	 * Send presence packet.
	 * This will force the controller to resend connection packets.
	 * This is useful in the case we activate the module after the
	 * adapter has been plugged in, as it won't automatically
	 * send us info about the controllers.
	 */
	error = xpad_inquiry_pad_presence(xpad);
	if (error) {
		usb_kill_urb(xpad->irq_in);
		return error;
	}

	printk(KERN_INFO "xpad360 finalizada");

	return 0;
}




static void xpad_irq_out(struct urb *urb)
{
	struct usb_xpad *xpad = urb->context;
	struct device *dev = &xpad->intf->dev;
	int status = urb->status;
	int error;
	unsigned long flags;

	spin_lock_irqsave(&xpad->odata_lock, flags);

	switch (status) {
	case 0:
		/* success */
		xpad->irq_out_active = xpad_prepare_next_out_packet(xpad);
		break;

	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(dev, "%s - urb shutting down with status: %d\n",
			__func__, status);
		xpad->irq_out_active = false;
		break;

	default:
		dev_dbg(dev, "%s - nonzero urb status received: %d\n",
			__func__, status);
		break;
	}

	if (xpad->irq_out_active) {
		usb_anchor_urb(urb, &xpad->irq_out_anchor);
		error = usb_submit_urb(urb, GFP_ATOMIC);
		if (error) {
			dev_err(dev,
				"%s - usb_submit_urb failed with result %d\n",
				__func__, error);
			usb_unanchor_urb(urb);
			xpad->irq_out_active = false;
		}
	}

	spin_unlock_irqrestore(&xpad->odata_lock, flags);
}



static int xpad_init_output(struct usb_interface *intf, struct usb_xpad *xpad,
			struct usb_endpoint_descriptor *ep_irq_out)
{
	int error;

	init_usb_anchor(&xpad->irq_out_anchor);

	xpad->odata = usb_alloc_coherent(xpad->udev, XPAD_PKT_LEN, GFP_KERNEL, &xpad->odata_dma);


	if (!xpad->odata)
		return -ENOMEM;

	spin_lock_init(&xpad->odata_lock);

	xpad->irq_out = usb_alloc_urb(0, GFP_KERNEL);
	if (!xpad->irq_out) {
		error = -ENOMEM;
	}

	usb_fill_int_urb(xpad->irq_out, xpad->udev,
			 usb_sndintpipe(xpad->udev, ep_irq_out->bEndpointAddress),
			 xpad->odata, XPAD_PKT_LEN,
			 xpad_irq_out, xpad, ep_irq_out->bInterval);
	xpad->irq_out->transfer_dma = xpad->odata_dma;
	xpad->irq_out->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	return 0;
}

void set_capability_keys(struct input_dev *input_dev){

	input_set_capability(input_dev, EV_KEY, KEY_LEFTCTRL);

	input_set_capability(input_dev, EV_KEY, KEY_C);
	input_set_capability(input_dev, EV_KEY, KEY_V);
	input_set_capability(input_dev, EV_KEY, KEY_Z);


	input_set_capability(input_dev, EV_KEY, KEY_A);
	input_set_capability(input_dev, EV_KEY, KEY_L);
	input_set_capability(input_dev, EV_KEY, KEY_I);
	input_set_capability(input_dev, EV_KEY, KEY_S);
	input_set_capability(input_dev, EV_KEY, KEY_O);
	input_set_capability(input_dev, EV_KEY, KEY_N);
	input_set_capability(input_dev, EV_KEY, KEY_K);
	input_set_capability(input_dev, EV_KEY, KEY_A);
	input_set_capability(input_dev, EV_KEY, KEY_M);
	input_set_capability(input_dev, EV_KEY, KEY_Y);
	input_set_capability(input_dev, EV_KEY, KEY_T);
	input_set_capability(input_dev, EV_KEY, KEY_P);
	input_set_capability(input_dev, EV_KEY, KEY_R);
	input_set_capability(input_dev, EV_KEY, KEY_C);


	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_RIGHT);

	input_set_capability(input_dev, EV_KEY, KEY_LEFTSHIFT);
	input_set_capability(input_dev, EV_KEY, KEY_BACKSPACE);
	input_set_capability(input_dev, EV_KEY, KEY_LEFTALT);
	input_set_capability(input_dev, EV_KEY, KEY_TAB);
	input_set_capability(input_dev, EV_KEY, KEY_ENTER);
	input_set_capability(input_dev, EV_KEY, KEY_SPACE);

}


// USB - Probe - Função de entrada quando um novo dispositivo é reconhecido para este modulo
static int meu_driver_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    int retval = 0;

    struct usb_xpad *minha_struct;
    struct usb_device *udev = interface_to_usbdev(interface);
    struct usb_endpoint_descriptor *ep_irq_in = NULL;
	struct input_dev *input_dev;


    minha_struct = kzalloc(sizeof(struct usb_xpad), GFP_KERNEL);
	if (!minha_struct)
		return -ENOMEM;

    // Verifica se é a interface possui 2 endpoints
    if (interface->cur_altsetting->desc.bNumEndpoints != 2)
		return -ENODEV;


	minha_struct->udev = udev;

    minha_struct->irq_in = usb_alloc_urb(0, GFP_KERNEL);

    minha_struct->idata = usb_alloc_coherent(udev, XPAD_PKT_LEN, GFP_KERNEL, &minha_struct->idata_dma);

    minha_struct->intf = interface;


    for (int i = 0; i < 2; i++){
        if (usb_endpoint_xfer_int(&(interface->cur_altsetting->endpoint[i].desc))){
            printk(KERN_INFO "O endpoint na posicao [%d] é interrupt", i);
        }
        if (usb_endpoint_dir_in(&(interface->cur_altsetting->endpoint[i].desc))){

            printk(KERN_INFO "O endpoint na posicao [%d] é IN", i);


            ep_irq_in = &interface->cur_altsetting->endpoint[i].desc;

            usb_fill_int_urb(minha_struct->irq_in, udev, usb_rcvintpipe(udev, ep_irq_in->bEndpointAddress),
                                minha_struct->idata, XPAD_PKT_LEN, xpad_irq_in,
                                minha_struct, ep_irq_in->bInterval);
        }

        if (usb_endpoint_dir_out(&(interface->cur_altsetting->endpoint[i].desc))){
            printk(KERN_INFO "O endpoint na posicao [%d] é OUT", i);
            xpad_init_output(interface, minha_struct, &(interface->cur_altsetting->endpoint[i].desc));
        }
    }

    minha_struct->irq_in->transfer_dma = minha_struct->idata_dma;
	minha_struct->irq_in->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;


    printk(KERN_INFO "meu_driver_usb: O dispositivo idVendor=%X idProduct=%X foi connectado ao meu driver, interface=%X", id->idVendor, id->idProduct, interface->cur_altsetting->desc.bInterfaceNumber);


	input_dev = input_allocate_device();

	minha_struct->dev = input_dev;

	sprintf(minha_struct->name, "X360");
	sprintf(minha_struct->phys, "/controle");

	input_dev->name = minha_struct->name;
	input_dev->phys = minha_struct->phys;

	usb_to_input_id(udev, &input_dev->id);

	input_set_drvdata(input_dev, minha_struct);

	//input_set_capability(input_dev, EV_KEY, KEY_S);
	set_capability_keys(input_dev);

	input_register_device(minha_struct->dev);

	usb_set_intfdata(interface, minha_struct);
	xpad360w_start_input(minha_struct);

    return retval;
}

// USB - Disconnect - Função de saída quando um dispositivo é desconectado
static void meu_driver_usb_disconnect(struct usb_interface *interface)
{
	struct usb_xpad *minha_struct = usb_get_intfdata(interface);

	printk(KERN_ALERT "Disconectando");

	if (minha_struct){
		input_unregister_device(minha_struct->dev);
		kfree(minha_struct);
	}
}

// USB - Tabela de dispositivos 
static struct usb_device_id minha_tabela_usb[] =
{
    { USB_DEVICE(0x045E, 0x028E) },
    {} // Entrada final
};
MODULE_DEVICE_TABLE (usb, minha_tabela_usb);

// USB - Definição do driver USB
static struct usb_driver meu_driver_usb =
{
    .name = "meu_driver_usb",
    .probe = meu_driver_usb_probe,
    .disconnect = meu_driver_usb_disconnect,
    .id_table = minha_tabela_usb,
};


// **********************
//   Controle do Módulo
// **********************

// Inicialização do modulo
static int __init meu_modulo_init(void)
{
    int result;

    // Registrando um dispositivo USB
    if ((result = usb_register(&meu_driver_usb)))
    {
        printk(KERN_ERR "Registro com erro: %d", result);
    }
    return result;
}

// Finalização do modulo
static void __exit meu_modulo_exit(void)
{
    // Desregistrando o dispositivo USB
    usb_deregister(&meu_driver_usb);
}

// Definição do modulo
// Entrypoint - A função que deve inicializar o modulo
module_init(meu_modulo_init);
// Exitpoint - A função que deve deinicializar o modulo
module_exit(meu_modulo_exit);

// Informações do Modulo
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elison Maiko, Italo Kusmin e Kamylo Porto");
MODULE_DESCRIPTION("Nosso modulo USB");
