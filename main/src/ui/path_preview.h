#ifndef PATH_PREVIEW_H
#define PATH_PREVIEW_H

/**
 * @brief Draws a scaled-down preview of the entire G-code path
 * loaded in the global 'path' variable onto the 'screen'.
 */
void drawPathPreview();

/**
 * @brief Clears all previously drawn path preview elements
 */
void clearPreviousPathPreview();

#endif