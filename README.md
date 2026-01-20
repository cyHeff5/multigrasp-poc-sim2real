# MultiGrasp – Sim2Real PoC

## Ziel

Dieses Projekt ist ein **Proof of Concept**, der prueft, ob ein in der Simulation gelernter  
**10-dimensionaler Greifvektor** fuer eine **AR10-Mehrfingerhand**  
**ohne manuelle Anpassung auf reale Hardware uebertragbar** ist.

Kurz gesagt:

> **Funktioniert ein simulierter Griff auf der echten Hand – ja oder nein?**

---

## Was hier passiert

1. In **PyBullet** wird eine Kombination aus **UR5-Arm + AR10-Hand** simuliert.  
2. Fuer **ein festes Benchmark-Objekt** wird pro Episode **genau ein 10D-Vektor** getestet.  
3. Die Hand nimmt diese Konfiguration ein, der Arm hebt das Objekt an.  
4. Der Versuch wird ueber einen **Reward** bewertet:
   - Kontakt → kleiner positiver Reward  
   - Objekt angehoben → grosser positiver Reward  
   - Nicht angehoben → negativer Reward  
   - Umgestossen → starker negativer Reward  
5. Der beste Vektor wird gespeichert und auf die **reale AR10-Hand** uebertragen.

---
