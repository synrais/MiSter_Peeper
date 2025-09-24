package main

import (
	"github.com/gdamore/tcell/v2"
	"github.com/rivo/tview"
	"log"
)

func main() {
	app := tview.NewApplication()

	// Create a simple list (menu)
	menu := tview.NewList().
		AddItem("Start", "Begin the process", 's', func() {
			showMessage(app, "Starting...")
		}).
		AddItem("Settings", "Change preferences", 't', func() {
			showMessage(app, "Opening settings...")
		}).
		AddItem("Help", "Show help information", 'h', func() {
			showMessage(app, "Here is the help text.")
		}).
		AddItem("Quit", "Exit the program", 'q', func() {
			app.Stop()
		})

	menu.SetBorder(true).SetTitle(" Main Menu ").SetTitleAlign(tview.AlignLeft)

	if err := app.SetRoot(menu, true).EnableMouse(true).Run(); err != nil {
		log.Fatal(err)
	}
}

func showMessage(app *tview.Application, msg string) {
	modal := tview.NewModal().
		SetText(msg).
		AddButtons([]string{"OK"}).
		SetDoneFunc(func(buttonIndex int, buttonLabel string) {
			app.SetRoot(mainMenu(app), true)
		})
	app.SetRoot(modal, true)
}

func mainMenu(app *tview.Application) *tview.List {
	menu := tview.NewList().
		AddItem("Start", "Begin the process", 's', func() {
			showMessage(app, "Starting...")
		}).
		AddItem("Settings", "Change preferences", 't', func() {
			showMessage(app, "Opening settings...")
		}).
		AddItem("Help", "Show help information", 'h', func() {
			showMessage(app, "Here is the help text.")
		}).
		AddItem("Quit", "Exit the program", 'q', func() {
			app.Stop()
		})

	menu.SetBorder(true).SetTitle(" Main Menu ").SetTitleAlign(tview.AlignLeft)
	return menu
}
