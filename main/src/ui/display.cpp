#include "display.h"
#include "../config.h"
#include "../globals.h"
#include "../math/geometry.h"
#include <Arduino.h>

static int16_t lastX0, lastY0, lastX1, lastY1, lastX2, lastY2, lastX3, lastY3;
static int16_t lastTargetCircleX, lastTargetCircleY;

unsigned long lastScrollTime = 0;
int scrollPosition = 0;
const int SCROLL_DELAY = 500;    // Time before scrolling starts (ms)
const int SCROLL_SPEED = 150;    // Time between each scroll step (ms)
const int CHARS_TO_DISPLAY = 12; // Max characters that fit on screen with text size 2

float exponentialSkew(float x) {
	if (x > 0) {
		return x + (1/exp(x));
	} else if (x < 0) {
		return -(x + (1/exp(x)));
	}
	return 0.0f;
}

void drawTypeMenu() {
	const char* options[] = {"Preset", "From File"};
	const int numOptions = 2;

	screen->fillScreen(BLACK);

	// Set text properties
	screen->setTextSize(2);
	screen->setTextColor(WHITE);
	
	// Calculate vertical spacing
	int16_t yStart = screen->height() / 3;
	int16_t ySpacing = 30;
	
	// Draw each option
	for (int i = 0; i < numOptions; i++) {
		// Highlight selected option
		if (i == designType) {
			screen->setTextColor(YELLOW);
		} else {
			screen->setTextColor(WHITE);
		}
		
		// Center text horizontally
		int16_t x1, y1;
		uint16_t w, h;
		screen->getTextBounds(options[i], 0, 0, &x1, &y1, &w, &h);
		int16_t x = (screen->width() - w) / 2;
		
		// Draw option text
		screen->setCursor(x, yStart + (i * ySpacing));
		screen->print(options[i]);
	}
}

void drawShape() {
	int16_t tftWidth = screen->width();
	int16_t tftHeight = screen->height();
	int16_t centerX = tftWidth / 2;
	int16_t centerY = tftHeight / 2;
	int16_t size = min(tftWidth, tftHeight) / 3;

	float scale;
	int16_t minY,maxY,minX,maxX,y_quarter,y_3_quarter,x;

	screen->fillScreen(BLACK);

	switch (designPreset) {
		// TODO: draw an accurate representation of the design here
		case 0:
			// line
			screen->drawLine(centerX, centerY-size, centerX, centerY+size, WHITE);
			break;
		case 1:
			// sin
			scale = size / PI;
			for (int y = -size; y <= size; y++) {
				x = (int16_t) (scale*sin(y/scale));
				screen->drawPixel(centerX+x, centerY+y, WHITE);
			}
			break;
		case 2:
			// zigzag
			minY = centerY - size;
			maxY = centerY + size;
			minX = centerX - size / 2;
			maxX = centerX + size / 2;

			y_quarter = minY + size / 2;
			y_3_quarter = maxY - size / 2;

			screen->drawLine(centerX, minY, maxX, y_quarter, WHITE);
			screen->drawLine(maxX, y_quarter, minX, y_3_quarter, WHITE);
			screen->drawLine(minX, y_3_quarter, centerX, maxY, WHITE);

			break;
		case 3:
			// double line
			screen->drawLine(centerX-size/4, centerY-size, centerX-size/4, centerY+size, WHITE);
			screen->drawLine(centerX+size/4, centerY-size, centerX+size/4, centerY+size, WHITE);
			break;
		case 4:
			// diamond
			screen->drawLine(centerX-size, centerY, centerX, centerY+size, WHITE);
			screen->drawLine(centerX, centerY+size, centerX+size, centerY, WHITE);
			screen->drawLine(centerX+size, centerY, centerX, centerY-size, WHITE);
			screen->drawLine(centerX, centerY-size, centerX-size, centerY, WHITE);
			break;
		case 5:
			// square w/ squiggly
			screen->drawLine(centerX-size, centerY, centerX, centerY+size, WHITE);
			screen->drawLine(centerX, centerY+size, centerX+size, centerY, WHITE);
			screen->drawLine(centerX+size, centerY, centerX, centerY-size, WHITE);
			screen->drawLine(centerX, centerY-size, centerX-size, centerY, WHITE);
			scale = size / PI;
			for (int y = -size; y <= size; y++) {
				x = (int16_t) (scale*sin(y/scale));
				screen->drawPixel(centerX+x, centerY+y, WHITE);
			}
			break;
		case 6:
			// square with Make
			drawCenteredText("M:",2);
			screen->drawLine(centerX-size, centerY, centerX, centerY+size, WHITE);
			screen->drawLine(centerX, centerY+size, centerX+size, centerY, WHITE);
			screen->drawLine(centerX+size, centerY, centerX, centerY-size, WHITE);
			screen->drawLine(centerX, centerY-size, centerX-size, centerY, WHITE);
			break;
		case 8:
			// circle
			screen->drawCircle(centerX, centerY, size, WHITE);
			break;
		// 	// hexagon
		// 	screen->drawLine(centerX, centerY+size, centerX+size*cos(M_PI/6), centerY-size*sin(M_PI/6), WHITE);
		// 	screen->drawLine(centerX+size*cos(M_PI/6), centerY-size*sin(M_PI/6), centerX+size*cos(M_PI/6), centerY-size*sin(M_PI/6), WHITE);
		// 	screen->drawLine(centerX+size*cos(M_PI/6), centerY-size*sin(M_PI/6), centerX, centerY-size, WHITE);
		// 	screen->drawLine(centerX, centerY-size, centerX-size*cos(M_PI/6), centerY-size*sin(M_PI/6), WHITE);
		// 	screen->drawLine(centerX-size*cos(M_PI/6), centerY-size*sin(M_PI/6), centerX-size*cos(M_PI/6), centerY+size*sin(M_PI/6), WHITE);
		// 	screen->drawLine(centerX-size*cos(M_PI/6), centerY+size*sin(M_PI/6), centerX, centerY+size, WHITE);
		// 	break;
	}
}

void listFiles() {
	screen->fillScreen(BLACK);
	screen->setTextSize(2);

	int16_t tftHeight = screen->height();

	// Calculate vertical centering
    int totalHeight = displayLines * 20;  				// Total height of all lines (7 lines * 20px)
    int startY = (tftHeight - totalHeight) / 2;  		// Center vertically on screen
	
	// Calculate which files to show to keep selection centered
	int startIndex = max(0, current_file_idx - centerLine);
	
	// Adjust start index if we're near the end of the list
	if (startIndex + displayLines > totalFiles) {
		startIndex = max(0, totalFiles - displayLines);
	}

	// Handle scrolling for selected item
    unsigned long currentTime = millis();
    if (currentTime - lastScrollTime > SCROLL_SPEED) {
        lastScrollTime = currentTime;
        scrollPosition++;
    }
	
	// Draw each visible line
	for (int i = 0; i < displayLines; i++) {
		int fileIndex = startIndex + i;
		if (fileIndex >= totalFiles) break;
		
		// Calculate Y position
		int y = startY + i * 20;  // Assuming 20 pixels per line with text size 2
		screen->setCursor(0, y);

		String displayText = fileList[fileIndex];
		
		// Set color based on selection
		if (fileIndex == current_file_idx) {
			screen->setTextColor(YELLOW);
			screen->print("> ");

			// Handle scrolling for long filename
            if (displayText.length() > CHARS_TO_DISPLAY) {
                // Add spaces at the end before repeating
                displayText = displayText + "    " + displayText;
                int totalScroll = displayText.length();
                int currentPos = scrollPosition % totalScroll;
                displayText = displayText.substring(currentPos, currentPos + CHARS_TO_DISPLAY);
            }
		} else {
			screen->setTextColor(WHITE);
			screen->print("  ");

			// Truncate non-selected long filenames
            if (displayText.length() > CHARS_TO_DISPLAY) {
                displayText = displayText.substring(0, CHARS_TO_DISPLAY - 3) + "...";
            }
		}
		
		// Print file/folder name
		screen->println(displayText);
	}

	// Reset scroll position when selection changes
    static int lastIndex = -1;
    if (lastIndex != current_file_idx) {
        scrollPosition = 0;
        lastScrollTime = currentTime + SCROLL_DELAY; // Add delay before scrolling starts
        lastIndex = current_file_idx;
    }
}

void updateFileList() {
	totalFiles = 0;
	
	// Clear previous list
	for (int i = 0; i < MAX_FILES; i++) {
		fileList[i] = "";
	}
	
	// Add parent directory entry if not in root
	char currentDirName[256];
	currentDir.getName(currentDirName, sizeof(currentDirName));
	if (strcmp(currentDirName, "/") != 0) {
		fileList[totalFiles++] = "../";
	}
	
	// First pass: add directories
	currentDir.rewindDirectory();
	while (FsFile entry = currentDir.openNextFile()) {
		if (totalFiles >= MAX_FILES) break;
		
		if (entry.isDirectory()) {
			char nameBuf[256];
			entry.getName(nameBuf, sizeof(nameBuf));
			fileList[totalFiles++] = String(nameBuf) + "/";
		}
		entry.close();
	}
	
	// Second pass: add files
	currentDir.rewindDirectory();
	while (FsFile entry = currentDir.openNextFile()) {
		if (totalFiles >= MAX_FILES) break;
		
		if (!entry.isDirectory()) {
			char nameBuf[256];
			entry.getName(nameBuf, sizeof(nameBuf));
			fileList[totalFiles++] = String(nameBuf);
		}
		entry.close();
	}
	
	currentDir.rewindDirectory();
}

void drawCenteredText(const char* text, int size) {
	Serial.print("SCREEN: ");
	Serial.println(text);

	screen->fillScreen(BLACK);

	int16_t tftWidth = screen->width();
	int16_t tftHeight = screen->height();
	int16_t centerX = tftWidth / 2;
	int16_t centerY = tftHeight / 2;

	screen->setFont(u8g2_font_littlemissloudonbold_tr);
	screen->setTextSize(size);
	screen->setTextColor(WHITE);

	// Split text into lines
	char *lines[10];
	int lineCount = 0;
	char textCopy[256];
	strncpy(textCopy, text, sizeof(textCopy) - 1);
	char *line = strtok(textCopy, "\n");
	
	while (line != NULL && lineCount < 10) {
		lines[lineCount++] = line;
		line = strtok(NULL, "\n");
	}

	int16_t totalHeight = lineCount * size * 10;
	int16_t yStart = centerY - totalHeight / 2;

	for (int i = 0; i < lineCount; i++) {
		int16_t x1, y1;
		uint16_t w, h;
		screen->getTextBounds(lines[i], 0, 0, &x1, &y1, &w, &h);
		int16_t xStart = centerX - w / 2;
		screen->setCursor(xStart, yStart + i * size * 10);
		screen->println(lines[i]);
	}
}

void drawFixedUI() {
	screen->fillScreen(BLACK);

	int16_t radius = screen->width()*0.95 / 2;
	int16_t centerX = screen->width() / 2;
	int16_t centerY = screen->width() / 2;

	// Draw arcs
	for (int i=0; i<radius; i++) {
		float xOffset = radius*cosf((PI/4)*i/radius);
		float yOffset = radius*sinf((PI/4)*i/radius);

		screen->drawPixel(centerX - xOffset, centerY - yOffset, WHITE);
		screen->drawPixel(centerX - xOffset, centerY + yOffset, WHITE);
		screen->drawPixel(centerX + xOffset, centerY - yOffset, WHITE);
		screen->drawPixel(centerX + xOffset, centerY + yOffset, WHITE);
	}
	
	// Draw bounds circle
	int16_t radiusBounds = screen->width() / 4;
	screen->drawCircle(centerX, centerY, radiusBounds, WHITE);
}

void drawUI(float desPosition, Point goal, Point next, uint8_t i) {
	int16_t radiusBounds = screen->width() / 4;
	int16_t radiusInner = screen->width() / 10;
	int16_t centerX = screen->width() / 2;
	int16_t centerY = screen->width() / 2;

	float dTheta = pose.yaw + PI/2 - atan2f(next.y-goal.y, next.x-goal.x);

	// float xMap = mapF(desPosition, -gantryLength/2, gantryLength/2, -radiusBounds, radiusBounds);
	// float yMap = mapF();
	float dx = 0.0f;
	float dy = 0.0f;
	if (paths[current_path_idx].feature != HOLE) {
		dx = (next.x-pose.x)*cosf(-pose.yaw) - (next.y-pose.y)*sinf(-pose.yaw);
		dy = (next.x-pose.x)*sinf(-pose.yaw) + (next.y-pose.y)*cosf(-pose.yaw);
	} else {
		dx = (goal.x-pose.x)*cosf(-pose.yaw) - (goal.y-pose.y)*sinf(-pose.yaw);
		dy = (goal.x-pose.x)*sinf(-pose.yaw) + (goal.y-pose.y)*cosf(-pose.yaw);
	}

	float dySkewed = 5*exponentialSkew(dy);
	// float theta = pose.yaw - atan2f(dySkewed, next.x-pose.x);
	float theta = atan2f(dySkewed, dx);
	// float thetaTool = pose.yaw - atan2f(next.y-(pose.y+motorPosX*sinf(pose.yaw)), next.x-(pose.x+motorPosX*cosf(pose.yaw)));
	// float dist = myDist(pose.x, pose.y, next.x, pose.y + dySkewed);
	float dist = sqrt(pow(dx,2)+pow(dySkewed,2));
	float holeDistance = abs(signedDist(pose, goal));

	// Serial.printf("yaw: %f\n", degrees(pose.yaw));

	float offsetRadius = radiusBounds*0.9*tanh(dist*0.05);
	//float offsetRadius = (radius - radiusInner)*0.8*tanh(dist*0.1);

	switch (i%8) {
		case 0:
			// draw the center target
			screen->drawLine(centerX, centerY-5, centerX, centerY+5, WHITE);
			screen->drawLine(centerX-15, centerY, centerX+15, centerY, WHITE);
			break;
		case 1:
			// draw outer up/down marker
			screen->drawLine(centerX+radiusBounds, centerY, centerX+radiusBounds-5, centerY, WHITE);
			screen->drawLine(centerX-radiusBounds, centerY, centerX-radiusBounds+5, centerY, WHITE);
			break;
		case 2:
			// clear the old line left
			screen->drawLine(lastX0, lastY0, lastX1, lastY1, BLACK);
			break;
		case 3:
			// clear old line right
			screen->drawLine(lastX2, lastY2, lastX3, lastY3, BLACK);
			break;
		case 4:
			// draw the new line left
			lastX0 = centerX + 1.4*radiusBounds*cosf(dTheta);
			lastY0 = centerY + 1.4*radiusBounds*sinf(dTheta);
			lastX1 = centerX + (radiusBounds+2)*cosf(dTheta);
			lastY1 = centerY + (radiusBounds+2)*sinf(dTheta);

			screen->drawLine(lastX0, lastY0, lastX1, lastY1, WHITE);
			break;
		case 5:
			// draw the new line right
			lastX2 = centerX - (radiusBounds+2)*cosf(dTheta);
			lastY2 = centerY - (radiusBounds+2)*sinf(dTheta);
			lastX3 = centerX - 1.4*radiusBounds*cosf(dTheta);
			lastY3 = centerY - 1.4*radiusBounds*sinf(dTheta);
			
			screen->drawLine(lastX2, lastY2, lastX3, lastY3, WHITE);
			break;
		case 6:
			// clear the old target circle
			screen->drawCircle(lastTargetCircleX, lastTargetCircleY, 5, BLACK);
			break;
		case 7:
			// draw new target circle
			// float toolX = radiusBounds*cosf(thetaTool);
			// float toolY = radiusBounds*sinf(thetaTool);
			lastTargetCircleX = centerX + (offsetRadius)*cosf(theta);
			lastTargetCircleY = centerY - (offsetRadius)*sinf(theta);     // TODO: this shouldn't be negative, just a hack
			// lastTargetCircleX = centerX + ((offsetRadius)*cosf(theta) + toolX)/2;
			// lastTargetCircleY = centerY + ((offsetRadius)*sinf(theta) + toolY)/2;
			// lastTargetCircleX = centerX + xMap;
			// lastTargetCircleY = centerY + radiusInner*sqrt((1-pow(xMap/radiusBounds,2)));
			if (paths[current_path_idx].feature != HOLE){
				screen->drawCircle(lastTargetCircleX, lastTargetCircleY, 5, WHITE);
			} else {
				if (holeDistance <= holeTolerance) {
					screen->drawCircle(lastTargetCircleX, lastTargetCircleY, 5, GREEN);
				} else if (holeDistance <= holeTolerance*10) {
					screen->drawCircle(lastTargetCircleX, lastTargetCircleY, 5, YELLOW);
				} else {
					screen->drawCircle(lastTargetCircleX, lastTargetCircleY, 5, RED);
				}
			}
			break;
	}
}

void drawDirection() {
	float width = 30;
	float height = screen->height() / 3;
	float spacing = 5;

	int16_t centerX = screen->height() / 2;
	int16_t centerY = screen->height() / 2;
	
	// Forward triangle
	int16_t x0 = centerX - width;
	int16_t y0 = centerY-spacing;
	int16_t x1 = centerX;
	int16_t y1 = centerY-height;
	int16_t x2 = centerX + width;
	int16_t y2 = centerY-spacing;

	// Backward triangle
	int16_t x3 = centerX - width;
	int16_t y3 = centerY+spacing;
	int16_t x4 = centerX;
	int16_t y4 = centerY+height;
	int16_t x5 = centerX + width;
	int16_t y5 = centerY+spacing;

	uint16_t forwardColor, reverseColor;

	if (paths[current_path_idx].direction > 0) {
		forwardColor = GC9A01A_WEBWORK_GREEN;
		reverseColor = DARKGREY;
	} else {
		forwardColor = DARKGREY;
		reverseColor = GC9A01A_WEBWORK_GREEN;
	}

	screen->drawTriangle(x0,y0,x1,y1,x2,y2,forwardColor);
	screen->drawTriangle(x3,y3,x4,y4,x5,y5,reverseColor);
}

void updateUI(float desPosition, Point goal, Point next) {
	if ((millis()-lastDraw) > 15) {
		iter = (iter + 1)%8;
		// unsigned long now = micros();
		// motorPosX = stepperX.currentPosition()*1.0f/Conv;
		drawUI(desPosition, goal, next, iter);
		// Serial.printf("draw %d took %i us\n", iter, micros()-now);
		lastDraw = millis();
	}
}
