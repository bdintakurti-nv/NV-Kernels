// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek power-wrap hooks for PCIe segments.
 *
 * Copyright (c) 2026 MediaTek Inc.
 */

#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/soc/mediatek/mtk-pwrap.h>
#include <linux/string.h>

#include "pci.h"

struct mtk_pci_pwrap_ctrl {
	void *dev_ctrl;
	char *acpi_path;
	struct pci_bus *root_bus;
	unsigned int port_count;
	unsigned int suspended_count;
	int dstate;
	struct list_head list;
};

struct mtk_pci_pwrap_port {
	struct pci_dev *pdev;
	struct mtk_pci_pwrap_ctrl *ctrl;
	struct list_head list;
	bool suspended;
	/* A failed noirq resume requires a reboot to restore PM ordering. */
	bool resume_noirq_failed;
};

static DEFINE_MUTEX(mtk_pci_pwrap_lock);
static LIST_HEAD(mtk_pci_pwrap_ctrls);
static LIST_HEAD(mtk_pci_pwrap_ports);

static char *mtk_pci_pwrap_get_host_path(struct pci_dev *pdev,
					 struct pci_bus **root_bus)
{
	struct pci_host_bridge *host;
	struct acpi_buffer buf = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_handle handle;
	acpi_status status;

	host = pci_find_host_bridge(pdev->bus);
	if (!host)
		return ERR_PTR(-ENODEV);
	*root_bus = host->bus;

	handle = ACPI_HANDLE(&host->dev);
	if (!handle)
		return ERR_PTR(-ENODEV);

	status = acpi_get_name(handle, ACPI_FULL_PATHNAME, &buf);
	if (ACPI_FAILURE(status))
		return ERR_PTR(-ENODEV);

	return buf.pointer;
}

static bool mtk_pci_pwrap_is_root_port(struct pci_dev *pdev)
{
	return pci_is_pcie(pdev) &&
	       pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT;
}

static bool mtk_pci_pwrap_should_manage(struct pci_dev *pdev)
{
	return mtk_pci_pwrap_is_root_port(pdev);
}

static bool mtk_pci_pwrap_keep_active(struct pci_dev *pdev,
				      bool system_transition)
{
	bool populated;

	/*
	 * During system suspend, keep a segment powered when the PCI core left
	 * the Root Port or a descendant in D0, or when an enabled wake source
	 * depends on the path.  Power Wrap physically removes the host clocks,
	 * power domains, and config-space access, so it must not override the
	 * core's skip_bus_pm propagation for a D0 hierarchy.
	 * Wake policy may change after probe, so evaluate it for every cycle.
	 */
	if (system_transition)
		return pdev->skip_bus_pm || device_may_wakeup(&pdev->dev) ||
		       device_wakeup_path(&pdev->dev);

	/* Preserve the existing conservative runtime-PM policy. */
	down_read(&pci_bus_sem);
	populated = pdev->subordinate &&
		    !list_empty(&pdev->subordinate->devices);
	up_read(&pci_bus_sem);

	return populated;
}

static struct mtk_pci_pwrap_port *mtk_pci_pwrap_find_port_locked(struct pci_dev *pdev)
{
	struct mtk_pci_pwrap_port *port;

	list_for_each_entry(port, &mtk_pci_pwrap_ports, list) {
		if (port->pdev == pdev)
			return port;
	}

	return NULL;
}

static struct mtk_pci_pwrap_port *
mtk_pci_pwrap_find_upstream_port_locked(struct pci_dev *pdev)
{
	struct pci_dev *root_port = pcie_find_root_port(pdev);

	if (!root_port)
		return NULL;

	return mtk_pci_pwrap_find_port_locked(root_port);
}

/*
 * Power Wrap gates the complete host segment.  Every function directly on
 * the root bus must therefore be one of this controller's suspended ports;
 * an untracked Root Port, RCiEP, RCEC, or endpoint must keep the segment in D0.
 */
static bool
mtk_pci_pwrap_host_ready_locked(struct pci_dev *pdev,
				struct mtk_pci_pwrap_ctrl *ctrl)
{
	struct pci_host_bridge *host = pci_find_host_bridge(pdev->bus);
	struct mtk_pci_pwrap_port *port;
	struct pci_dev *child;
	unsigned int count = 0;
	bool ready = false;

	if (!host || !host->bus)
		return false;

	list_for_each_entry(child, &host->bus->devices, bus_list) {
		port = mtk_pci_pwrap_find_port_locked(child);
		if (!port || port->ctrl != ctrl || !port->suspended)
			return false;
		count++;
	}

	ready = count == ctrl->port_count;
	return ready;
}

static struct mtk_pci_pwrap_ctrl *mtk_pci_pwrap_find_ctrl_locked(const char *path)
{
	struct mtk_pci_pwrap_ctrl *ctrl;

	list_for_each_entry(ctrl, &mtk_pci_pwrap_ctrls, list) {
		if (!strcmp(ctrl->acpi_path, path))
			return ctrl;
	}

	return NULL;
}

static void mtk_pci_pwrap_free_ctrl_locked(struct mtk_pci_pwrap_ctrl *ctrl)
{
	list_del(&ctrl->list);
	ACPI_FREE(ctrl->acpi_path);
	kfree(ctrl);
}

static void mtk_pci_pwrap_put_ctrl_locked(struct mtk_pci_pwrap_ctrl *ctrl,
					  struct device *dev)
{
	int ret;

	if (--ctrl->port_count)
		return;

	ret = mtk_pwrap_dev_remove(ctrl->dev_ctrl);
	if (ret)
		dev_warn(dev, "pwrap remove failed: %d\n", ret);

	mtk_pci_pwrap_free_ctrl_locked(ctrl);
}

static void mtk_pci_pwrap_release_port(void *data)
{
	struct mtk_pci_pwrap_port *port = data;
	struct mtk_pci_pwrap_ctrl *ctrl = port->ctrl;

	mutex_lock(&mtk_pci_pwrap_lock);
	if (port->suspended && ctrl->suspended_count)
		ctrl->suspended_count--;
	list_del(&port->list);
	mtk_pci_pwrap_put_ctrl_locked(ctrl, &port->pdev->dev);
	mutex_unlock(&mtk_pci_pwrap_lock);
}

void mtk_pci_pwrap_init(struct pci_dev *pdev)
{
	struct mtk_pci_pwrap_ctrl *node;
	struct mtk_pci_pwrap_port *port;
	struct pci_bus *root_bus;
	char *path;
	int ret;

	if (!mtk_pci_pwrap_should_manage(pdev))
		return;

	path = mtk_pci_pwrap_get_host_path(pdev, &root_bus);
	if (IS_ERR(path)) {
		dev_dbg(&pdev->dev, "failed to get ACPI host path: %ld\n",
			PTR_ERR(path));
		return;
	}

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port) {
		ACPI_FREE(path);
		return;
	}

	mutex_lock(&mtk_pci_pwrap_lock);
	if (mtk_pci_pwrap_find_port_locked(pdev))
		goto out_unlock;

	node = mtk_pci_pwrap_find_ctrl_locked(path);
	if (!node) {
		node = kzalloc_obj(*node);
		if (!node)
			goto out_unlock;

		node->acpi_path = path;
		path = NULL;
		node->root_bus = root_bus;
		node->dstate = DEV_STA_UNKNOWN;
		node->dev_ctrl = mtk_pwrap_dev_probe(node->acpi_path);
		if (!node->dev_ctrl) {
			dev_dbg(&pdev->dev, "no pwrap config for %s\n",
				node->acpi_path);
			ACPI_FREE(node->acpi_path);
			kfree(node);
			goto out_unlock;
		}

		node->dstate = DEV_STA_D0;
		list_add(&node->list, &mtk_pci_pwrap_ctrls);
	}

	port->pdev = pdev;
	port->ctrl = node;
	list_add(&port->list, &mtk_pci_pwrap_ports);
	node->port_count++;

	ret = devm_add_action(&pdev->dev, mtk_pci_pwrap_release_port, port);
	if (ret) {
		list_del(&port->list);
		mtk_pci_pwrap_put_ctrl_locked(node, &pdev->dev);
		dev_warn(&pdev->dev, "pwrap: cleanup registration failed: %d\n", ret);
	}

out_unlock:
	mutex_unlock(&mtk_pci_pwrap_lock);
	ACPI_FREE(path);
}

bool mtk_pci_pwrap_is_managed(struct pci_dev *pdev)
{
	bool managed;

	mutex_lock(&mtk_pci_pwrap_lock);
	managed = !!mtk_pci_pwrap_find_port_locked(pdev);
	mutex_unlock(&mtk_pci_pwrap_lock);

	return managed;
}

int mtk_pci_pwrap_suspend(struct pci_dev *pdev, bool system_transition)
{
	struct mtk_pci_pwrap_ctrl *node;
	struct mtk_pci_pwrap_port *port;
	bool rescan_locked;
	int ret = 0;

	if (!mtk_pci_pwrap_should_manage(pdev))
		return 0;
	if (mtk_pci_pwrap_keep_active(pdev, system_transition))
		return 0;

	/*
	 * A rescan probes configuration space before publishing new devices to
	 * the bus list.  Keep the segment in D0 throughout that window.  Use a
	 * trylock because device removal holds this lock while runtime PM may
	 * re-enter this path.
	 */
	rescan_locked = mutex_trylock(&pci_rescan_remove_lock);
	mutex_lock(&mtk_pci_pwrap_lock);
	port = mtk_pci_pwrap_find_port_locked(pdev);
	if (!port)
		goto out_unlock;

	node = port->ctrl;
	if (!node->dev_ctrl || port->suspended || port->resume_noirq_failed)
		goto out_unlock;

	port->suspended = true;
	node->suspended_count++;

	if (node->suspended_count != node->port_count)
		goto out_unlock;
	if (!rescan_locked)
		goto out_unlock;

	/* Serialize topology validation and segment gating with PCI hotplug. */
	down_read(&pci_bus_sem);
	if (!mtk_pci_pwrap_host_ready_locked(pdev, node))
		goto out_topology;

	if (node->dstate != DEV_STA_D0)
		goto out_topology;

	ret = mtk_pwrap_dev_suspend(node->dev_ctrl, system_transition);
	if (ret) {
		if (ret == -ETIMEDOUT) {
			dev_warn(&pdev->dev,
				 "pwrap suspend timed out; assuming D3\n");
			node->dstate = DEV_STA_D3;
			ret = 0;
		} else {
			dev_warn(&pdev->dev, "pwrap suspend failed: %d\n", ret);
			port->suspended = false;
			node->suspended_count--;
		}
		goto out_topology;
	}

	node->dstate = DEV_STA_D3;

out_topology:
	up_read(&pci_bus_sem);
out_unlock:
	mutex_unlock(&mtk_pci_pwrap_lock);
	if (rescan_locked)
		mutex_unlock(&pci_rescan_remove_lock);
	return ret;
}

static void
mtk_pci_pwrap_reconcile_system_resume_locked(struct mtk_pci_pwrap_ctrl *ctrl)
{
	struct mtk_pci_pwrap_port *port;

	list_for_each_entry(port, &mtk_pci_pwrap_ports, list) {
		if (port->ctrl == ctrl)
			port->suspended = false;
	}
	/* Keep the failure sticky: the port missed non-replayable noirq work. */
	ctrl->suspended_count = 0;
}

int mtk_pci_pwrap_resume(struct pci_dev *pdev, bool system_transition)
{
	struct mtk_pci_pwrap_ctrl *node;
	struct mtk_pci_pwrap_port *port;
	int ret = 0;

	mutex_lock(&mtk_pci_pwrap_lock);
	port = mtk_pci_pwrap_find_port_locked(pdev);
	if (!port)
		goto out_unlock;

	node = port->ctrl;
	if (!node->dev_ctrl || port->resume_noirq_failed) {
		ret = port->resume_noirq_failed ? -EIO : 0;
		goto out_unlock;
	}
	if (node->dstate != DEV_STA_D0) {
		ret = mtk_pwrap_dev_resume(node->dev_ctrl, system_transition);
		if (ret) {
			dev_warn(&pdev->dev, "pwrap resume failed: %d\n", ret);
			goto out_unlock;
		}
		node->dstate = DEV_STA_D0;
	}

	if (system_transition) {
		/* One controller request restores configuration for every port. */
		mtk_pci_pwrap_reconcile_system_resume_locked(node);
	} else if (port->suspended) {
		port->suspended = false;
		if (node->suspended_count)
			node->suspended_count--;
	}

out_unlock:
	mutex_unlock(&mtk_pci_pwrap_lock);
	return ret;
}

bool mtk_pci_pwrap_resume_noirq_failed(struct pci_dev *pdev)
{
	struct mtk_pci_pwrap_port *port;
	bool failed = false;

	mutex_lock(&mtk_pci_pwrap_lock);
	port = mtk_pci_pwrap_find_upstream_port_locked(pdev);
	if (port)
		failed = port->resume_noirq_failed;
	mutex_unlock(&mtk_pci_pwrap_lock);

	return failed;
}

void mtk_pci_pwrap_mark_resume_noirq_failed(struct pci_dev *pdev)
{
	struct mtk_pci_pwrap_port *port;
	struct mtk_pci_pwrap_port *sibling;
	struct mtk_pci_pwrap_ctrl *ctrl;
	struct pci_bus *root_bus = NULL;
	struct pci_dev *root_port;

	root_port = pcie_find_root_port(pdev);
	if (!root_port)
		return;

	mutex_lock(&mtk_pci_pwrap_lock);
	port = mtk_pci_pwrap_find_port_locked(root_port);
	if (!port)
		goto out_unlock;

	ctrl = port->ctrl;
	list_for_each_entry(sibling, &mtk_pci_pwrap_ports, list) {
		if (sibling->ctrl != ctrl || sibling->resume_noirq_failed)
			continue;

		sibling->resume_noirq_failed = true;
		if (!root_bus)
			root_bus = pci_bus_get(ctrl->root_bus);
	}

out_unlock:
	mutex_unlock(&mtk_pci_pwrap_lock);

	if (!root_bus)
		return;

	/* Power Wrap gates the complete host, so every hierarchy is unsafe. */
	pci_walk_bus(root_bus, pci_dev_set_disconnected, NULL);
	dev_err(&root_bus->dev,
		"pwrap D0 failed in resume_noirq; disconnecting host\n");
	pci_bus_put(root_bus);
}

static struct pci_bus *mtk_pci_pwrap_root_bus(struct pci_bus *bus)
{
	while (bus && !pci_is_root_bus(bus))
		bus = bus->parent;

	return bus;
}

int mtk_pci_pwrap_rescan_prepare(struct pci_bus *bus,
				 struct mtk_pci_pwrap_rescan_context *context)
{
	struct pci_bus *root_bus = mtk_pci_pwrap_root_bus(bus);
	struct mtk_pci_pwrap_port *port;
	unsigned int count = 0;
	unsigned int index = 0;
	int ret = 0;

	lockdep_assert_held(&pci_rescan_remove_lock);
	memset(context, 0, sizeof(*context));

	mutex_lock(&mtk_pci_pwrap_lock);
	list_for_each_entry(port, &mtk_pci_pwrap_ports, list) {
		if (bus && port->ctrl->root_bus != root_bus)
			continue;
		/* A segment containing a failed port cannot be rescanned safely. */
		if (port->resume_noirq_failed) {
			ret = -EIO;
			goto out_unlock;
		}
		count++;
	}
	if (!count)
		goto out_unlock;

	context->anchors = kcalloc(count, sizeof(*context->anchors), GFP_KERNEL);
	if (!context->anchors) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	/*
	 * A controller-wide D0 request restores ECAM, but each Root Port has
	 * independent PCI runtime-PM state.  Hold every matching port active so
	 * a subordinate-bus rescan cannot probe through a sibling left in D3.
	 */
	list_for_each_entry(port, &mtk_pci_pwrap_ports, list) {
		if (bus && port->ctrl->root_bus != root_bus)
			continue;
		context->anchors[index++] = pci_dev_get(port->pdev);
	}
	context->anchor_count = index;

out_unlock:
	mutex_unlock(&mtk_pci_pwrap_lock);
	if (ret)
		return ret;

	for (index = 0; index < context->anchor_count; index++) {
		struct pci_dev *anchor = context->anchors[index];

		ret = pm_runtime_resume_and_get(&anchor->dev);
		if (ret < 0) {
			dev_warn(&anchor->dev,
				 "failed to resume pwrap segment for rescan: %d\n",
				 ret);
			return ret;
		}
		context->active_count++;

		/* Confirm segment D0 even when runtime PM was disabled. */
		ret = mtk_pci_pwrap_resume(anchor, false);
		if (ret) {
			dev_warn(&anchor->dev,
				 "failed to restore pwrap segment for rescan: %d\n",
				 ret);
			return ret;
		}
	}

	return 0;
}

void mtk_pci_pwrap_rescan_done(struct mtk_pci_pwrap_rescan_context *context)
{
	unsigned int index;

	for (index = 0; index < context->active_count; index++) {
		pm_runtime_mark_last_busy(&context->anchors[index]->dev);
		pm_runtime_put_autosuspend(&context->anchors[index]->dev);
	}
	for (index = 0; index < context->anchor_count; index++)
		pci_dev_put(context->anchors[index]);

	kfree(context->anchors);
	memset(context, 0, sizeof(*context));
}
