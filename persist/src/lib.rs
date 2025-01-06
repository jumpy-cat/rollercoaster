use serde::{Deserialize, Serialize};

#[test]
fn migrate() {
    let data = serde_json::from_str::<DataLegacy>(include_str!("../../hi2.json")).unwrap();
    println!("{:#?}", data);
    let migrated = Data {
        points: data
            .positions
            .into_iter()
            .map(|pos| PersistPoint {
                pos,
                deriv: None,
                optimizer_can_adjust_pos: false,
            })
            .collect(),
    };
    println!("{}", serde_json::to_string_pretty(&migrated).unwrap());
}

#[derive(Serialize, Deserialize, Debug)]
pub struct PersistPoint {
    pub pos: [f64; 3],
    pub deriv: Option<[[f64; 3]; 3]>,
    pub optimizer_can_adjust_pos: bool,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Data {
    pub points: Vec<PersistPoint>,
}

#[derive(Serialize, Deserialize, Debug)]
struct DataLegacy {
    positions: Vec<[f64; 3]>,
}
