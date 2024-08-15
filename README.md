# renesas-rza

Projekt állapota:

Az átküldött zip fájl majdnem 1-1be máködőképes volt, 2 dolgot kellett javítani:
- Linux miatt az include file name kisbetű-nagybetű mismatch
- A projekt beállítások között a egyes útvonalak rosszul voltak. Ezt vagy az xml hekkelésével, vagy a GUI segítségével kell javítani. 

    > **FIGYELEM!** Ez után is rosszul leszek, mert `/home/akos` van, úgy néz ki nem tud relatívet

brachek:

- `main`: én gépemen fordult és működő projekt
- `test_rotate`: a forgatás tesztelése
- `test_rotate_updated`: a forgatás tesztelése a 2 odjára elküldött projekten. lejebb indul ki hogy közvetlenül össze lehessen hasonlítani a változtatásokat a két zipen, még nem tartalmaz szubmodult (chekoutkor megparad az előző)