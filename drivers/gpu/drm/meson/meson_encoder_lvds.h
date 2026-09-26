/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Amlogic Meson8m2 LVDS encoder
 */

#ifndef __MESON_ENCODER_LVDS_H
#define __MESON_ENCODER_LVDS_H

struct meson_drm;

int meson_encoder_lvds_probe(struct meson_drm *priv);
void meson_encoder_lvds_remove(struct meson_drm *priv);

#endif /* __MESON_ENCODER_LVDS_H */
