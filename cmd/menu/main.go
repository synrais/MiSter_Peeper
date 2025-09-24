package main

import (
	"log"

	"github.com/rivo/tview"
)

// Global lists
var (
	systemList *tview.List
	gameList   *tview.List
)

func main() {
	app := tview.NewApplication()

	// Initialize lists
	systemList = tview.NewList().
		AddItem("NES", "Nintendo Entertainment System", 'n', func() {
			loadGames(app, "NES")
		}).
		AddItem("SNES", "Super Nintendo", 's', func() {
			loadGames(app, "SNES")
		}).
		AddItem("Genesis", "Sega Genesis / Mega Drive", 'g', func() {
			loadGames(app, "Genesis")
		})

	gameList = tview.NewList()

	// Layout: systems on left, games on right
	layout := tview.NewFlex().
		AddItem(systemList, 0, 1, true).
		AddItem(gameList, 0, 2, false)

	if err := app.SetRoot(layout, true).EnableMouse(true).Run(); err != nil {
		log.Fatal(err)
	}
}

// loadGames fills the game list based on system
func loadGames(app *tview.Application, system string) {
	gameList.Clear()

	switch system {
	case "NES":
		gameList.AddItem("Super Mario Bros.", "", 0, func() {
			showMessage(app, "Starting Super Mario Bros. (NES)")
		})
		gameList.AddItem("The Legend of Zelda", "", 0, func() {
			showMessage(app, "Starting Zelda (NES)")
		})

	case "SNES":
		gameList.AddItem("Super Mario World", "", 0, func() {
			showMessage(app, "Starting Super Mario World (SNES)")
		})
		gameList.AddItem("Donkey Kong Country", "", 0, func() {
			showMessage(app, "Starting Donkey Kong Country (SNES)")
		})

	case "Genesis":
		gameList.AddItem("Sonic the Hedgehog", "", 0, func() {
			showMessage(app, "Starting Sonic (Genesis)")
		})
		gameList.AddItem("Streets of Rage", "", 0, func() {
			showMessage(app, "Starting Streets of Rage (Genesis)")
		})
	}
}

// showMessage displays a simple modal
func showMessage(app *tview.Application, msg string) {
	modal := tview.NewModal().
		SetText(msg).
		AddButtons([]string{"OK"}).
		SetDoneFunc(func(buttonIndex int, buttonLabel string) {
			// Return to the system/game view
			layout := tview.NewFlex().
				AddItem(systemList, 0, 1, true).
				AddItem(gameList, 0, 2, false)
			app.SetRoot(layout, true)
		})
	app.SetRoot(modal, true)
}
