#include <linux/netdevice.h> 
#include <linux/etherdevice.h> 
#include <linux/skbuff.h> 
#include <linux/ethtool.h> 
 
struct net_device_stats mulintdev_stats;   /* Statistics */ 
 
/* Fill ethtool_ops methods from a suitable place in the driver */ 
struct ethtool_ops mulint_ethtool_ops = { 
	
}; 

/* Initialize/probe the card. For PCI cards, this is invoked 
   from (or is itself) the probe() method. In that case, the 
   function is declared as: 
   static struct net_device *init_mycard(struct pci_dev *pdev, const 
                                         struct pci_device_id *id) 
*/ 

static struct net_device *mulintdev[4];

struct mulint_priv {
	int ethX;
};

static int mulintdev_ndo_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	struct cpsw_priv *priv = netdev_priv(dev);
	int slave_no = cpsw_slave_index(priv);

	if (!netif_running(dev))
		return -EINVAL;

	switch (cmd) {
#ifdef CONFIG_TI_CPTS
	case SIOCSHWTSTAMP:
		return cpsw_hwtstamp_set(dev, req);
	case SIOCGHWTSTAMP:
		return cpsw_hwtstamp_get(dev, req);
#endif
	}

	if (!priv->slaves[slave_no].phy)
		return -EOPNOTSUPP;
	return phy_mii_ioctl(priv->slaves[slave_no].phy, req, cmd);
}
static int mulintdev_ndo_set_mac_address(struct net_device *netdev, void *p)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	struct sockaddr *addr = (struct sockaddr *)p;
	int flags = 0;
	u16 vid = 0;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	if (priv->data.dual_emac) {
		vid = priv->slaves[priv->emac_port].port_vlan;
		flags = ALE_VLAN;
	}

	cpsw_ale_del_ucast(priv->ale, priv->mac_addr, priv->host_port,
			   flags, vid);
	cpsw_ale_add_ucast(priv->ale, addr->sa_data, priv->host_port,
			   flags, vid);

	memcpy(priv->mac_addr, addr->sa_data, ETH_ALEN);
	memcpy(ndev->dev_addr, priv->mac_addr, ETH_ALEN);
	for_each_slave(priv, cpsw_set_slave_mac, priv);

	return 0;
}

static netdev_tx_t mulintdev_ndo_start_xmit(struct sk_buff *skb,
				       struct net_device *netdev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	int ret;
	int len;

	ndev->trans_start = jiffies;

	if (skb_padto(skb, CPSW_MIN_PACKET_SIZE)) {
		cpsw_err(priv, tx_err, "packet pad failed\n");
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP &&
				priv->cpts->tx_enable)
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

	skb_tx_timestamp(skb);
	len = max(skb->len, CPSW_MIN_PACKET_SIZE);
	netdev_sent_queue(ndev,len);
	ret = cpsw_tx_packet_submit(ndev, priv, skb);
	if (unlikely(ret != 0)) {
		netdev_completed_queue(ndev,1,len);
		cpsw_err(priv, tx_err, "desc submit failed\n");
		goto fail;
	}

	/* If there is no more tx desc left free then we need to
	 * tell the kernel to stop sending us tx frames.
	 */
	if (unlikely(!cpdma_check_free_tx_desc(priv->txch)))
		netif_stop_queue(ndev);

	return NETDEV_TX_OK;
fail:
	ndev->stats.tx_dropped++;
	netif_stop_queue(ndev);
	return NETDEV_TX_BUSY;
}

static int mulintdev_ndo_stop(struct net_device *netdev)
{
	return 0;
}

static void mulintdev_ndo_tx_timeout(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	cpsw_err(priv, tx_err, "transmit timeout, restarting dma\n");
	ndev->stats.tx_errors++;
	cpsw_intr_disable(priv);
	cpdma_ctlr_int_ctrl(priv->dma, false);
	cpdma_chan_stop(priv->txch);
	cpdma_chan_start(priv->txch);
	cpdma_ctlr_int_ctrl(priv->dma, true);
	cpsw_intr_enable(priv);
}

static void mulintdev_ndo_set_rx_mode(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	int vid;

	if (priv->data.dual_emac)
		vid = priv->slaves[priv->emac_port].port_vlan;
	else
		vid = priv->data.default_vlan;

	if (ndev->flags & IFF_PROMISC) {
		/* Enable promiscuous mode */
		cpsw_set_promiscious(ndev, true);
		cpsw_ale_set_allmulti(priv->ale, IFF_ALLMULTI);
		return;
	} else {
		/* Disable promiscuous mode */
		cpsw_set_promiscious(ndev, false);
	}

	/* Restore allmulti on vlans if necessary */
	cpsw_ale_set_allmulti(priv->ale, priv->ndev->flags & IFF_ALLMULTI);

	/* Clear all mcast from ALE */
	cpsw_ale_flush_multicast(priv->ale, ALE_ALL_PORTS << priv->host_port,
				 vid);

	if (!netdev_mc_empty(ndev)) {
		struct netdev_hw_addr *ha;

		/* program multicast address list into ALE register */
		netdev_for_each_mc_addr(ha, ndev) {
			cpsw_add_mcast(priv, (u8 *)ha->addr);
		}
	}
}



static const struct net_device_ops mulint_netdev_ops = {
	.ndo_open		= mulintdev_ndo_open,
	.ndo_stop		= mulintdev_ndo_stop,
	.ndo_start_xmit		= mulintdev_ndo_start_xmit,
	.ndo_set_mac_address	= mulintdev_ndo_set_mac_address,
	.ndo_do_ioctl		= mulintdev_ndo_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_tx_timeout		= mulintdev_ndo_tx_timeout,
	.ndo_set_rx_mode	= mulintdev_ndo_set_rx_mode,
};

static struct net_device * 
init_mulintdev(int int_num) 
{ 
    int err;
    
    struct net_device *netdev;
    struct mulint_priv *priv;

    char mac[4][6]={{0xa0,0x00,0x00,0x11,0x11,0x11},
                {0xa0,0x00,0x00,0x22,0x22,0x22},
                {0xa0,0x00,0x00,0x33,0x33,0x33},
                {0xa0,0x00,0x00,0x44,0x44,0x44}};

    /* ... */ 
    netdev = alloc_etherdev(sizeof(struct mulint_priv)); 
    
    /* Data transfer */
    netdev->watchdog_timeo = 8*HZ;             /* 8-second timeout */ 
    
    netdev->netdev_ops = &mulint_netdev_ops;
    /* Statistics and configuration */ 
    netdev->ethtool_ops = &mulint_ethtool_ops; /* Ethtool support */ 
    netdev->stats =

    memcpy(netdev->dev_addr, mac[int_num], 6);
    memcpy(netdev->name,"mulgibit",8);
    

    
    priv = netdev_priv(dev); 
    
    /* Register the interface */ 
    err = register_netdev(netdev); 
    if (err)
    {
        free_netdev(netdev);
        return NULL;
    }
    
    /* ... */ 
   
    /* Get MAC address from attached EEPROM */ 
    /* ... */ 
   
    /* Download microcode if needed */ 
    /* ... */ 
} 
 
 
/* The interrupt handler */ 
static irqreturn_t 
mulintdev_interrupt(int irq, void *dev_id) 
{ 
  struct net_device *netdev = dev_id; 
  struct sk_buff *skb; 
  unsigned int length; 
 
  /* ... */ 
 
  if (receive_interrupt) { 
    /* We were interrupted due to packet reception. At this point, 
       the NIC has already DMA'ed received data to an sk_buff that 
       was pre-allocated and mapped during device open. Obtain the 
       address of the sk_buff depending on your data structure 
       design and assign it to 'skb'. 'length' is similarly obtained 
       from the NIC by reading the descriptor used to DMA data from 
       the card. Now, skb->data contains the receive data. */ 
    /* ... */
/* For PCI cards, perform a pci_unmap_single() on the 
       received buffer in order to allow the CPU to access it */ 
    /* ... */ 
 
    /* Allow the data go to the tail of the packet by moving 
       skb->tail down by length bytes and increasing 
       skb->len correspondingly */ 
    skb_put(skb, length) 
 
    /* Pass the packet to the TCP/IP stack */ 
#if !defined (USE_NAPI)  /* Do it the old way */ 
    netif_rx(skb); 
#else                    /* Do it the NAPI way */ 
    if (netif_rx_schedule_prep(netdev))) { 
      /* Disable NIC interrupt. Implementation not shown. */ 
      disable_nic_interrupt(); 
 
      /* Post the packet to the protocol layer and 
         add the device to the poll list */ 
      __netif_rx_schedule(netdev); 
    } 
#endif 
  } else if (tx_complete_interrupt) { 
    /* Transmit Complete Interrupt */ 
    /* ... */ 
    /* Unmap and free transmit resources such as 
       DMA descriptors and buffers. Free sk_buffs or 
       reclaim them into a free pool */ 
    /* ... */ 
 
 } 
} 
 
/* Driver open */ 
static int 
mulintdev_ndo_open(struct net_device *netdev) 
{ 
  /* ... */ 
 
  /* Request irq */ 
  request_irq(irq, mycard_interrupt, IRQF_SHARED, 
              netdev->name, dev); 
 
  /* Allocate Descriptor rings */ 
  /* See the section, "Buffer Management and Concurrency Control" */ 
  /* ... */ 
 
  /* Provide free descriptor addresses to the card */ 
  /* ... */ 
 
  /* Convey your readiness to accept data from the 
     networking stack */ 
   netif_start_queue(netdev);
	 /* ... */ 
} 
 
/* Driver close */ 
static int 
mulintdev_close(struct net_device *netdev) 
{ 
  /* ... */ 
 
  /* Ask the networking stack to stop sending down data */ 
   netif_stop_queue(netdev); 
 
  /* ... */ 
} 
/* Called when the device is unplugged or when the module is 
   released. For PCI cards, this is invoked from (or is itself) 
   the remove() method. In that case, the function is declared as: 
   static void __devexit mycard_remove(struct pci_dev *pdev) 
*/ 
static void __devexit 
mulintdev_remove() 
{ 
  struct net_device *netdev; 
 
  /* ... */ 
  /* For a PCI card, obtain the associated netdev as follows, 
     assuming that the probe() method performed a corresponding 
     pci_set_drvdata(pdev, netdev) after allocating the netdev */ 
  netdev = pci_get_drvdata(pdev); /* 
 
  unregister_netdev(netdev);  /* Reverse of register_netdev() */ 
 
  /* ... */ 
 
  free_netdev(netdev);       /* Reverse of alloc_netdev() */ 
 
  /* ... */ 
} 
 
 
/* Suspend method. For PCI devices, this is part of 
   the pci_driver structure discussed in Chapter 10 */ 
static int 
mulintdev_suspend(struct pci_dev *pdev, pm_message_t state) 
{ 
  /* ... */ 
  netif_device_detach(netdev); 
  /* ... */ 
} 
/* Resume method. For PCI devices, this is part of 
   the pci_driver structure discussed in Chapter 10 */ 
static int 
mulintdev_resume(struct pci_dev *pdev)
{ 
  /* ... */ 
  netif_device_attach(netdev); 
  /* ... */ 
} 
 
/* Get statistics */ 
static struct net_device_stats * 
mulintdev_get_stats(struct net_device *netdev) 
{ 
  /* House keeping */ 
  /* ... */ 
 
  return(&mulintdev_stats); 
} 
 
/* Dump EEPROM contents. This is an ethtool_ops operation */ 
static int 
mulintdev_get_eeprom(struct net_device *netdev, 
                  struct ethtool_eeprom *eeprom, uint8_t *bytes) 
{ 
  /* Read data from the accompanying EEPROM */ 
  /* ... */ 
} 
 
/* Poll method */ 
static int 
mulintdev_poll(struct net_device *netdev, int *budget) 
{ 
   /* Post packets to the protocol layer using 
      netif_receive_skb() */ 
   /* ... */ 
 
   if (no_more_ingress_packets()){ 
     /* Remove the device from the polled list */ 
     netif_rx_complete(netdev); 
 
     /* Fall back to interrupt mode. Implementation not shown */ 
     enable_nic_interrupt(); 
 
     return 0; 
  } 
} 
/* Transmit method */ 
static int 
mulintdev_xmit_frame(struct sk_buff *skb, struct net_device *netdev) 
{ 
  /* DMA the transmit packet from the associated sk_buff to card memory */ 
  /* ... */ 
  /* Manage buffers */ 
  /* ... */ 
}


static int __init mulintdev_init(void)
{
    int i=0;
    for (i=0;i<4;i++)
    {
        mulintdev[i]  =  init_mulintdev(i);
    }
}
module_init(mulintdev_init);

static void __exit mulintdev_exit(void)
{
	
}
module_exit(mulintdev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LuMen <luomin5417@gmail.com>");
MODULE_DESCRIPTION("bb-black Mulint Dev Ethernet driver");
